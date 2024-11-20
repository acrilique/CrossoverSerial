#define USE_ARM_DSP
#include "daisy_pod.h"
#include "daisy_seed.h"
#include "daisysp.h"
#include "../desktopProgrammer/CrossoverProtocol.h"
#include "iir.h"
#include <string.h>
#include <cmath>

using namespace daisy;
using namespace daisysp;

static DaisyPod hw;
static DaisySeed seed;

// Device identification
static constexpr char DEVICE_NAME[] = "DAISY_CROSSOVER";
static constexpr uint32_t DEVICE_ID = 0;

// Buffer for incoming serial data
static uint8_t serialBuffer[Protocol::MAX_MESSAGE_SIZE];
static size_t serialBufferIndex = 0;

// IIR filter objects - one per channel
static IIR<IIRFILTER_USER_MEMORY> filter_left;
static IIR<IIRFILTER_USER_MEMORY> filter_right;

// State buffers - 4 states per stage, up to 16 stages
static float filter_states_left[64];  // 16 stages * 4 states
static float filter_states_right[64]; // 16 stages * 4 states

// Coefficients buffer - 5 coeffs per stage, up to 16 stages
static float filter_coeffs[80];       // 16 stages * 5 coefficients

// Current filter settings
static Protocol::FilterParameters filterParams[16];
static uint8_t numActiveFilters = 0;

// Settings struct for persistent storage
struct Settings {
    Protocol::FilterParameters filterParams[16];
    uint8_t numActiveFilters;

    bool operator!=(const Settings& a) const {
        if (a.numActiveFilters != numActiveFilters) return true;
        for(int i = 0; i < numActiveFilters; i++) {
            if(a.filterParams[i].enabled != filterParams[i].enabled ||
               a.filterParams[i].frequency != filterParams[i].frequency ||
               a.filterParams[i].q != filterParams[i].q ||
               a.filterParams[i].gain != filterParams[i].gain ||
               a.filterParams[i].type != filterParams[i].type) {
                return true;
            }
        }
        return false;
    }
};

// Persistent Storage Declaration
static PersistentStorage<Settings> storage(hw.seed.qspi);

void CalculateFilterCoefficients(const Protocol::FilterParameters& params, float sampleRate, float* coeffs) {
    float w0 = 2.0f * M_PI * params.frequency / sampleRate;
    float alpha = sinf(w0) / (2.0f * params.q);
    float A = powf(10.0f, params.gain / 40.0f);
    float cos_w0 = cosf(w0);
    float a0 = 1.0f;
    
    // Initialize coefficients with default values
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;

    switch (params.type) {
        case Protocol::FilterType::PeakingEQ:
            b0 = 1.0f + alpha * A;
            b1 = -2.0f * cos_w0;
            b2 = 1.0f - alpha * A;
            a0 = 1.0f + alpha / A;
            a1 = -2.0f * cos_w0;
            a2 = 1.0f - alpha / A;
            break;

        case Protocol::FilterType::LowShelf:
            {
                float sqrtA = sqrtf(A);
                a0 = (A + 1.0f) + (A - 1.0f) * cos_w0 + 2.0f * sqrtA * alpha;
                b0 = A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + 2.0f * sqrtA * alpha);
                b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0);
                b2 = A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - 2.0f * sqrtA * alpha);
                a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0);
                a2 = (A + 1.0f) + (A - 1.0f) * cos_w0 - 2.0f * sqrtA * alpha;
            }
            break;

        case Protocol::FilterType::HighShelf:
            {
                float sqrtA = sqrtf(A);
                a0 = (A + 1.0f) - (A - 1.0f) * cos_w0 + 2.0f * sqrtA * alpha;
                b0 = A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + 2.0f * sqrtA * alpha);
                b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0);
                b2 = A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - 2.0f * sqrtA * alpha);
                a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0);
                a2 = (A + 1.0f) - (A - 1.0f) * cos_w0 - 2.0f * sqrtA * alpha;
            }
            break;

        case Protocol::FilterType::HighPass:
            b0 = (1.0f + cos_w0) / 2.0f;
            b1 = -(1.0f + cos_w0);
            b2 = (1.0f + cos_w0) / 2.0f;
            a0 = 1.0f + alpha;
            a1 = -2.0f * cos_w0;
            a2 = 1.0f - alpha;
            break;

        case Protocol::FilterType::LowPass:
            b0 = (1.0f - cos_w0) / 2.0f;
            b1 = 1.0f - cos_w0;
            b2 = (1.0f - cos_w0) / 2.0f;
            a0 = 1.0f + alpha;
            a1 = -2.0f * cos_w0;
            a2 = 1.0f - alpha;
            break;
    }

    // Normalize coefficients by a0 and store in ARM CMSIS DSP biquad format
    // [b0/a0, b1/a0, b2/a0, -a1/a0, -a2/a0]
    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = -a1 / a0;
    coeffs[4] = -a2 / a0;
}

void InitializeFilters() {
    // Initialize one filter per channel with state buffers large enough for all stages
    filter_left.SetStateBuffer(filter_states_left, 64);
    filter_right.SetStateBuffer(filter_states_right, 64);
    
    // Reset filters to ensure clean state
    filter_left.Reset();
    filter_right.Reset();
}

bool UpdateFilters() {
    float sample_rate = hw.AudioSampleRate();
    uint8_t activeStages = 0;
    
    // Reset filters first to ensure clean state
    filter_left.Reset();
    filter_right.Reset();
    
    // If no active filters, just return true (passthrough mode)
    if (numActiveFilters == 0) {
        return true;
    }
    
    // Validate numActiveFilters
    if (numActiveFilters > 16) {
        numActiveFilters = 16;  // Safety clamp
    }
    
    // Calculate coefficients for each active filter
    for (int i = 0; i < numActiveFilters && activeStages < 16; i++) {
        if (filterParams[i].enabled) {
            CalculateFilterCoefficients(filterParams[i], sample_rate, &filter_coeffs[activeStages * 5]);
            activeStages++;
        }
    }
    
    // Only initialize the ARM DSP biquad filter if we have active stages
    if (activeStages > 0) {
        bool left_success = filter_left.SetIIR(filter_coeffs, activeStages);
        bool right_success = filter_right.SetIIR(filter_coeffs, activeStages);
        return left_success && right_success;
    }
    
    return true;
}

void LoadSettings() {
    Settings& stored = storage.GetSettings();
    memcpy(filterParams, stored.filterParams, sizeof(filterParams));
    numActiveFilters = stored.numActiveFilters;
    
    // Ensure filters are in a known state before updating
    filter_left.Reset();
    filter_right.Reset();
    
    if (!UpdateFilters()) {
        // If filter update fails, reset to default state
        numActiveFilters = 0;
        memset(filterParams, 0, sizeof(filterParams));
        UpdateFilters();
    }
}

void SaveSettings() {
    Settings& stored = storage.GetSettings();
    memcpy(stored.filterParams, filterParams, sizeof(filterParams));
    stored.numActiveFilters = numActiveFilters;
    storage.Save();
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    if (numActiveFilters > 0) {
        filter_left.ProcessBlock(const_cast<float*>(in[0]), out[0], size);
        filter_right.ProcessBlock(const_cast<float*>(in[1]), out[1], size);
    } else {
        // Direct passthrough when no filters are active
        memcpy(out[0], in[0], size * sizeof(float));
        memcpy(out[1], in[1], size * sizeof(float));
    }
}

void SendMessage(Protocol::MessageType type, const void* payload, uint16_t payloadSize)
{
    uint8_t buffer[Protocol::MAX_MESSAGE_SIZE];
    size_t messageSize = sizeof(Protocol::MessageHeader) + payloadSize + sizeof(Protocol::MessageFooter);
    
    // Prepare header
    Protocol::MessageHeader* header = reinterpret_cast<Protocol::MessageHeader*>(buffer);
    header->startMarker1 = 0xAA;
    header->startMarker2 = 0x55;
    header->protocolVersion = Protocol::PROTOCOL_VERSION;
    header->type = type;
    header->payloadLength = payloadSize;
    
    // Copy payload if any
    if (payload && payloadSize > 0) {
        memcpy(buffer + sizeof(Protocol::MessageHeader), payload, payloadSize);
    }
    
    // Calculate checksum
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < sizeof(Protocol::MessageHeader) + payloadSize; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    // Add footer
    Protocol::MessageFooter* footer = reinterpret_cast<Protocol::MessageFooter*>(
        buffer + sizeof(Protocol::MessageHeader) + payloadSize
    );
    footer->checksum = crc;
    footer->endMarker1 = 0x55;
    footer->endMarker2 = 0xAA;
    
    // Send message
    hw.seed.usb_handle.TransmitInternal(buffer, messageSize);
}

void ProcessMessage(const Protocol::MessageHeader* header, const uint8_t* payload)
{
    // Check protocol version
    if (header->protocolVersion != Protocol::PROTOCOL_VERSION) {
        Protocol::ErrorMessage error;
        error.code = Protocol::ErrorCode::INVALID_PROTOCOL_VERSION;
        strcpy(error.message, "Protocol version mismatch");
        SendMessage(Protocol::MessageType::ERROR, &error, sizeof(error));
        return;
    }
    
    switch (header->type)
    {
        case Protocol::MessageType::HANDSHAKE_REQUEST:
        {
            if (header->payloadLength != sizeof(Protocol::HandshakeRequest)) {
                break;
            }
            
            // Send handshake response
            Protocol::HandshakeResponse response;
            strncpy(response.deviceName, DEVICE_NAME, sizeof(response.deviceName) - 1);
            response.deviceId = DEVICE_ID;
            
            SendMessage(Protocol::MessageType::HANDSHAKE_RESPONSE, 
                       &response, sizeof(response));
            break;
        }
        
        case Protocol::MessageType::PING:
        {
            // Respond with PONG
            SendMessage(Protocol::MessageType::PONG, nullptr, 0);
            break;
        }
        
        case Protocol::MessageType::FILTER_PARAMETERS:
        {
            if (header->payloadLength != sizeof(Protocol::FilterParametersPayload)) {
                Protocol::ErrorMessage error;
                error.code = Protocol::ErrorCode::INVALID_MESSAGE_TYPE;
                strcpy(error.message, "Invalid filter parameters size");
                SendMessage(Protocol::MessageType::ERROR, &error, sizeof(error));
                break;
            }

            const Protocol::FilterParametersPayload* params = 
                reinterpret_cast<const Protocol::FilterParametersPayload*>(payload);

            // Update number of active filters and their parameters
            numActiveFilters = params->numBands;
            if (numActiveFilters > 16) numActiveFilters = 16;  // Safety check
            
            memcpy(filterParams, params->bands, numActiveFilters * sizeof(Protocol::FilterParameters));

            // Update filters
            if (!UpdateFilters()) {
                Protocol::ErrorMessage error;
                error.code = Protocol::ErrorCode::INVALID_FILTER_PARAMS;
                strcpy(error.message, "Failed to update filters");
                SendMessage(Protocol::MessageType::ERROR, &error, sizeof(error));
                // Reset filters and set passthrough mode
                numActiveFilters = 0;
                memset(filterParams, 0, sizeof(filterParams));
                UpdateFilters();
                break;
            }

            // Save new parameters to flash
            SaveSettings();

            // Send acknowledgment
            SendMessage(Protocol::MessageType::ACKNOWLEDGMENT, nullptr, 0);
            break;
        }
        
        default:
        {
            Protocol::ErrorMessage error;
            error.code = Protocol::ErrorCode::INVALID_MESSAGE_TYPE;
            strcpy(error.message, "Unknown message type");
            SendMessage(Protocol::MessageType::ERROR, &error, sizeof(error));
            break;
        }
    }
}

void UsbCallback(uint8_t* buf, uint32_t* len)
{
    for(size_t i = 0; i < *len && serialBufferIndex < sizeof(serialBuffer); i++)
    {
        serialBuffer[serialBufferIndex++] = buf[i];
        
        // Look for complete message
        if (serialBufferIndex >= sizeof(Protocol::MessageHeader))
        {
            const Protocol::MessageHeader* header = 
                reinterpret_cast<Protocol::MessageHeader*>(serialBuffer);
                
            // Check start markers
            if (header->startMarker1 != 0xAA || header->startMarker2 != 0x55)
            {
                // Invalid start markers, remove first byte and continue
                memmove(serialBuffer, serialBuffer + 1, --serialBufferIndex);
                continue;
            }
            
            // Check if we have complete message
            size_t messageSize = sizeof(Protocol::MessageHeader) + 
                               header->payloadLength + 
                               sizeof(Protocol::MessageFooter);
                               
            if (serialBufferIndex >= messageSize)
            {
                // Calculate checksum
                uint16_t crc = 0xFFFF;
                for (size_t j = 0; j < sizeof(Protocol::MessageHeader) + header->payloadLength; j++) {
                    crc ^= serialBuffer[j];
                    for (int k = 0; k < 8; k++) {
                        if (crc & 0x0001) {
                            crc = (crc >> 1) ^ 0xA001;
                        } else {
                            crc = crc >> 1;
                        }
                    }
                }

                const Protocol::MessageFooter* footer = 
                    reinterpret_cast<const Protocol::MessageFooter*>(
                        serialBuffer + sizeof(Protocol::MessageHeader) + header->payloadLength
                    );
                    
                if (crc == footer->checksum &&
                    footer->endMarker1 == 0x55 && 
                    footer->endMarker2 == 0xAA)
                {
                    // Process valid message
                    ProcessMessage(header, 
                                 serialBuffer + sizeof(Protocol::MessageHeader));
                }
                
                // Remove processed message from buffer
                if (serialBufferIndex > messageSize)
                {
                    memmove(serialBuffer, 
                           serialBuffer + messageSize,
                           serialBufferIndex - messageSize);
                }
                serialBufferIndex -= messageSize;
            }
        }
    }
}

int main(void)
{
    // Initialize hardware
    hw.Init();
    hw.SetAudioBlockSize(4); // number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    // Initialize persistent storage with default values first
    Settings defaultSettings = {};  // Zero-initialize all fields
    storage.Init(defaultSettings);

    // Initialize filters after storage but before loading settings
    InitializeFilters();

    // Load saved settings if they exist
    LoadSettings();

    // Start audio after all initialization is complete
    hw.StartAudio(AudioCallback);

    // Initialize USB
    hw.seed.usb_handle.Init(UsbHandle::FS_INTERNAL);
    hw.seed.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_INTERNAL);

    // Turn on LED to indicate we're ready
    hw.seed.SetLed(true);
    
    while(1)
    {
        hw.DelayMs(1);
    }
}
