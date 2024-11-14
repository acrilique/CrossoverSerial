#include "daisy_seed.h"
#include "daisysp.h"
#include "../desktopProgrammer/CrossoverProtocol.h"
#include <string.h>

using namespace daisy;
using namespace daisysp;

static DaisySeed hw;

// Device identification
static constexpr char DEVICE_NAME[] = "DAISY_CROSSOVER";
static constexpr uint32_t DEVICE_ID = 0x12345678;

// Buffer for incoming serial data
static uint8_t serialBuffer[Protocol::MAX_MESSAGE_SIZE];
static size_t serialBufferIndex = 0;

// Filter objects
static Svf parametricEQ_left[8];
static Svf parametricEQ_right[8];
static Svf highpass_left;
static Svf highpass_right;
static Svf lowpass_left;
static Svf lowpass_right;

// Current filter settings
static Protocol::FilterParameters eqParams[8];
static Protocol::HighpassFilter hpfParams;
static Protocol::LowpassFilter lpfParams;

void InitializeFilters() {
    float sample_rate = hw.AudioSampleRate();
    for (int i = 0; i < 8; i++) {
        parametricEQ_left[i].Init(sample_rate);
        parametricEQ_right[i].Init(sample_rate);
    }
    highpass_left.Init(sample_rate);
    highpass_right.Init(sample_rate);
    lowpass_left.Init(sample_rate);
    lowpass_right.Init(sample_rate);
}

void UpdateFilters() {
    for (int i = 0; i < 8; i++) {
        if (eqParams[i].enabled) {
            parametricEQ_left[i].SetFreq(eqParams[i].frequency);
            parametricEQ_left[i].SetRes(1.0f / eqParams[i].q);
            parametricEQ_right[i].SetFreq(eqParams[i].frequency);
            parametricEQ_right[i].SetRes(1.0f / eqParams[i].q);
        }
    }

    if (hpfParams.enabled) {
        highpass_left.SetFreq(hpfParams.frequency);
        highpass_left.SetRes(1.0f / hpfParams.q);
        highpass_right.SetFreq(hpfParams.frequency);
        highpass_right.SetRes(1.0f / hpfParams.q);
    }

    if (lpfParams.enabled) {
        lowpass_left.SetFreq(lpfParams.frequency);
        lowpass_left.SetRes(1.0f / lpfParams.q);
        lowpass_right.SetFreq(lpfParams.frequency);
        lowpass_right.SetRes(1.0f / lpfParams.q);
    }
}

float ApplyGain(float input, float gainDB) {
    return input * powf(10.0f, gainDB / 20.0f);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        float left = in[0][i];
        float right = in[1][i];

        // Apply filters
        if (hpfParams.enabled) {
            highpass_left.Process(left);
            left = highpass_left.High();
            highpass_right.Process(right);
            right = highpass_right.High();
        }

        for (int j = 0; j < 8; j++) {
            if (eqParams[j].enabled) {
                parametricEQ_left[j].Process(left);
                left = ApplyGain(parametricEQ_left[j].Peak(), eqParams[j].gain);
                parametricEQ_right[j].Process(right);
                right = ApplyGain(parametricEQ_right[j].Peak(), eqParams[j].gain);
            }
        }

        if (lpfParams.enabled) {
            lowpass_left.Process(left);
            left = lowpass_left.Low();
            lowpass_right.Process(right);
            right = lowpass_right.Low();
        }

        out[0][i] = left;
        out[1][i] = right;
    }
}

void SendMessage(Protocol::MessageType type, const void* payload, uint16_t payloadSize)
{
    uint8_t buffer[Protocol::MAX_MESSAGE_SIZE];
    size_t messageSize = sizeof(Protocol::MessageHeader) + payloadSize + sizeof(Protocol::MessageFooter);
    
    // Prepare header
    Protocol::MessageHeader* header = reinterpret_cast<Protocol::MessageHeader*>(buffer);
    header->startMarker1 = 0xAA;  // Explicitly set start markers
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
    hw.usb_handle.TransmitInternal(buffer, messageSize);
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

            // Update filter parameters
            memcpy(eqParams, params->bands, sizeof(eqParams));
            memcpy(&hpfParams, &params->hpf, sizeof(hpfParams));
            memcpy(&lpfParams, &params->lpf, sizeof(lpfParams));

            // Update filters
            UpdateFilters();

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
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4); // number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.StartAudio(AudioCallback);

    InitializeFilters();

    // Initialize USB
    hw.usb_handle.Init(UsbHandle::FS_BOTH);
    hw.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_BOTH);

    // Turn on LED to indicate we're ready
    hw.SetLed(true);
    
    while(1)
    {
        hw.DelayMs(1);
    }
}
