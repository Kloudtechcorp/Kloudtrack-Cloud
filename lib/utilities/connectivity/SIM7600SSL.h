/**
 * SIM7600 Native SSL/TLS Support
 *
 * Uses SIM7600's built-in SSL capabilities via AT commands
 * instead of software SSL (SSLClient library)
 */

#ifndef SIM7600SSL_H
#define SIM7600SSL_H

#include <Arduino.h>
#include <TinyGsmClient.h>

class SIM7600SSL {
private:
    TinyGsm* modem;
    HardwareSerial* serial;
    bool sslConfigured;
    int contextId;

public:
    SIM7600SSL(TinyGsm* modemPtr, HardwareSerial* serialPtr, int ctx = 1)
        : modem(modemPtr), serial(serialPtr), sslConfigured(false), contextId(ctx) {}

    /**
     * Configure SSL certificates for SIM7600
     * Stores certificates in modem's file system
     */
    bool configureCertificates(const char* caCert, const char* deviceCert, const char* privateKey) {
        Serial.println("[SIM7600SSL] Configuring SSL certificates...");

        // Set SSL version to TLS 1.2
        if (!sendATCommand("AT+CSSLCFG=\"sslversion\",1,3", 1000)) {
            Serial.println("[SIM7600SSL] Failed to set SSL version");
            return false;
        }

        // Enable server authentication
        if (!sendATCommand("AT+CSSLCFG=\"authmode\",1,2", 1000)) {
            Serial.println("[SIM7600SSL] Failed to set auth mode");
            return false;
        }

        // Convert and store CA certificate
        if (!storeCertificate("cacert.pem", caCert)) {
            Serial.println("[SIM7600SSL] Failed to store CA cert");
            return false;
        }

        // Convert and store device certificate
        if (!storeCertificate("clientcert.pem", deviceCert)) {
            Serial.println("[SIM7600SSL] Failed to store device cert");
            return false;
        }

        // Convert and store private key
        if (!storeCertificate("clientkey.pem", privateKey)) {
            Serial.println("[SIM7600SSL] Failed to store private key");
            return false;
        }

        // Configure certificate paths
        String cmd = "AT+CSSLCFG=\"cacert\"," + String(contextId) + ",\"cacert.pem\"";
        if (!sendATCommand(cmd.c_str(), 1000)) {
            Serial.println("[SIM7600SSL] Failed to set CA cert path");
            return false;
        }

        cmd = "AT+CSSLCFG=\"clientcert\"," + String(contextId) + ",\"clientcert.pem\"";
        if (!sendATCommand(cmd.c_str(), 1000)) {
            Serial.println("[SIM7600SSL] Failed to set client cert path");
            return false;
        }

        cmd = "AT+CSSLCFG=\"clientkey\"," + String(contextId) + ",\"clientkey.pem\"";
        if (!sendATCommand(cmd.c_str(), 1000)) {
            Serial.println("[SIM7600SSL] Failed to set client key path");
            return false;
        }

        sslConfigured = true;
        Serial.println("[SIM7600SSL] SSL configuration complete");
        return true;
    }

    /**
     * Connect to SSL server using modem's native SSL
     */
    bool connect(const char* host, int port) {
        if (!sslConfigured) {
            Serial.println("[SIM7600SSL] SSL not configured");
            return false;
        }

        Serial.printf("[SIM7600SSL] Connecting to %s:%d\n", host, port);

        // Start SSL service
        String cmd = "AT+CIPSSL=1";
        if (!sendATCommand(cmd.c_str(), 2000)) {
            Serial.println("[SIM7600SSL] Failed to enable SSL");
            return false;
        }

        // Connect to server
        cmd = "AT+CIPSTART=\"TCP\",\"" + String(host) + "\"," + String(port);
        if (!sendATCommand(cmd.c_str(), 30000, "CONNECT")) {
            Serial.println("[SIM7600SSL] Failed to connect");
            return false;
        }

        Serial.println("[SIM7600SSL] SSL connection established");
        return true;
    }

    void disconnect() {
        sendATCommand("AT+CIPCLOSE", 2000);
    }

private:
    bool sendATCommand(const char* cmd, unsigned long timeout, const char* expected = "OK") {
        serial->println(cmd);
        unsigned long start = millis();
        String response = "";

        while (millis() - start < timeout) {
            if (serial->available()) {
                char c = serial->read();
                response += c;

                if (response.indexOf(expected) >= 0) {
                    return true;
                }

                if (response.indexOf("ERROR") >= 0) {
                    Serial.printf("[SIM7600SSL] AT command error: %s\n", cmd);
                    Serial.printf("Response: %s\n", response.c_str());
                    return false;
                }
            }
        }

        Serial.printf("[SIM7600SSL] AT command timeout: %s\n", cmd);
        return false;
    }

    bool storeCertificate(const char* filename, const char* certData) {
        int certLen = strlen(certData);

        // Delete existing file
        String cmd = "AT+FSDEL=\"" + String(filename) + "\"";
        sendATCommand(cmd.c_str(), 1000); // Don't check result, file may not exist

        // Create new file
        cmd = "AT+FSCREATE=\"" + String(filename) + "\"";
        if (!sendATCommand(cmd.c_str(), 2000)) {
            return false;
        }

        // Write certificate data in chunks (max 1024 bytes per write)
        const int chunkSize = 1024;
        int offset = 0;

        while (offset < certLen) {
            int writeSize = min(chunkSize, certLen - offset);

            cmd = "AT+FSWRITE=\"" + String(filename) + "\"," + String(offset) + "," +
                  String(writeSize) + ",10";
            serial->println(cmd);
            delay(100);

            // Wait for '>' prompt
            unsigned long start = millis();
            while (millis() - start < 2000) {
                if (serial->available() && serial->read() == '>') {
                    break;
                }
            }

            // Write data
            serial->write((const uint8_t*)(certData + offset), writeSize);
            delay(500);

            // Wait for OK
            start = millis();
            String response = "";
            bool success = false;
            while (millis() - start < 3000) {
                if (serial->available()) {
                    response += (char)serial->read();
                    if (response.indexOf("OK") >= 0) {
                        success = true;
                        break;
                    }
                }
            }

            if (!success) {
                Serial.printf("[SIM7600SSL] Failed to write chunk at offset %d\n", offset);
                return false;
            }

            offset += writeSize;
        }

        Serial.printf("[SIM7600SSL] Stored %s (%d bytes)\n", filename, certLen);
        return true;
    }
};

#endif // SIM7600SSL_H
