#include "netsim.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>

using std::max;
using std::min;
using std::vector;

// CRC-32 via basic modulo-2 division, MSB-first.
// Polynomial: x^32 + x^26 + ... + 1 = 0x04C11DB7
static uint32_t compute_crc32(const uint8_t *data, int len) {
    uint32_t r = 0;
    for (int i = 0; i < len; i++) {
        r ^= (uint32_t)data[i] << 24;
        for (int b = 0; b < 8; b++)
            r = (r & 0x80000000u) ? (r << 1) ^ 0x04C11DB7u : (r << 1);
    }
    return r;
}

// Optimal payload size given BER estimate.
// Minimises E[cost per data byte] = (P + H + K) / (P * p_ack)
// where p_ack = exp(-8*BER*(P+H)), H=6 (frame overhead), K=250.
// Closed-form solution of derivative = 0:
//   P_opt = (-c + sqrt(c^2 + 4c/(8*BER))) / 2,  c = K + H = 256
static int optimal_payload(double ber) {
    if (ber <= 0.0) return 65535;
    const double c = 256.0; // K(250) + H(6)
    double P = (-c + sqrt(c * c + 4.0 * c / (8.0 * ber))) / 2.0;
    return max(1, min(65535, (int)P));
}

int main(int argc, char *argv[]) {
    if (argc < 2) return 1;

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) return 1;

    fseek(fp, 0, SEEK_END);
    long fsz = ftell(fp);
    rewind(fp);

    vector<uint8_t> buf((size_t)fsz);
    if ((long)fread(buf.data(), 1, (size_t)fsz, fp) != fsz) {
        fclose(fp);
        return 1;
    }
    fclose(fp);

    // Pre-allocate max frame buffer (2 + 65535 + 4 bytes)
    vector<uint8_t> frame(2 + 65535 + 4);

    double ber = 1e-4;

    // Sliding window of last WIN transmissions for BER estimation
    const int WIN = 128;
    double win_bits[WIN] = {};
    int    win_nak[WIN]  = {};
    int    win_idx   = 0;
    int    win_count = 0;

    long pos = 0;
    while (pos < fsz) {
        int P = min(min(optimal_payload(ber), 20000), (int)(fsz - pos));

        // Build frame: [size 2B big-endian][payload P bytes][CRC 4B big-endian]
        frame[0] = (uint8_t)(P >> 8);
        frame[1] = (uint8_t)(P & 0xFF);
        memcpy(frame.data() + 2, buf.data() + pos, (size_t)P);
        uint32_t crc = compute_crc32(frame.data(), 2 + P);
        frame[2 + P + 0] = (uint8_t)(crc >> 24);
        frame[2 + P + 1] = (uint8_t)(crc >> 16);
        frame[2 + P + 2] = (uint8_t)(crc >> 8);
        frame[2 + P + 3] = (uint8_t)(crc);

        int result = send_frame(frame.data(), 2 + P + 4);
        if (result == NETSIM_ERROR) return 1;

        // Record this transmission in the sliding window
        win_bits[win_idx] = (double)(8 * (P + 4));
        win_nak[win_idx]  = (result == NETSIM_NAK) ? 1 : 0;
        win_idx = (win_idx + 1) % WIN;
        if (win_count < WIN) win_count++;

        if (result == NETSIM_ACK)
            pos += P;
        else
            ber = min(0.1, ber * 4.0);

        // Update BER estimate once we have at least 4 samples
        if (win_count >= 4) {
            double total_bits = 0.0;
            int    total_naks = 0;
            for (int i = 0; i < win_count; i++) {
                total_bits += win_bits[i];
                total_naks += win_nak[i];
            }
            double avg_bits = total_bits / win_count;
            double nak_rate = (double)total_naks / win_count;

            if (nak_rate > 0.0) {
                // p_nak = 1 - exp(-BER * avg_bits)  →  BER = -ln(1-nak_rate)/avg_bits
                double nr = min(nak_rate, 0.999);
                ber = -log(1.0 - nr) / avg_bits;
                ber = max(1e-9, min(0.1, ber));
            } else {
                // All ACKs: channel is better than estimated, reduce BER
                ber = max(1e-9, ber * 0.7);
            }
        }
    }

    return 0;
}
