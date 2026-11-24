/*
 * ============================================================================
 * FIRMWARE TRANSMITTER MULTIPROTOCOL + CUSTOM RX UNIVERSAL HYBRID (MICROZONE)
 * ============================================================================
 * Berbasis       : goebish/nrf24_multipro + Integrasi Protokol CustomRX (FHSS)
 * Ditujukan Untuk:
 *   1. Drone mainan pabrikan berbasis NRF24L01 / XN297 (Eachine E010, H8,
 *      Syma, Bayang, WLToys, CX-10, dll.)
 *   2. Receiver (RX) Custom Universal Hybrid buatan sendiri (Pesawat Normal,
 *      Elevon/Delta, Diferensial 2 Motor, dan Mobil RC Propo dengan FHSS 4-Ch,
 *      ESC Brushless / MOSFET Brushed, serta Telemetri Baterai 1S).
 *
 * ============================================================================
 * PANDUAN WIRING HARDWARE (ARDUINO PRO MINI / NANO ATmega328P 16MHz)
 * ============================================================================
 * 1. MODUL RADIO NRF24L01 (Software SPI - BUKAN pin D11/D12/D13!):
 *    - VCC  -> Output 3.3V Regulator Eksternal (AMS1117-3.3V) + Elco 10uF & 100nF
 *              (WAJIB 3.3V! Jangan sambung ke 5V atau pin 3V3 bawaan Nano!)
 *    - GND  -> GND Bersama (Arduino, Regulator 3.3V, dan Remote Microzone)
 *    - MOSI -> Pin D3
 *    - SCK  -> Pin D4
 *    - CE   -> Pin D5
 *    - MISO -> Pin A0
 *    - CSN  -> Pin A1
 *    - IRQ  -> Kosongkan (Tidak dipakai)
 *
 * 2. REMOTE CONTROL MICROZONE MC6C (Sinyal PPM):
 *    - Sinyal PPM -> Pin D2 (Pasang resistor Pull-Up 10k Ohm dari D2 ke VCC)
 *    - GND Remote -> GND Arduino (Common Ground)
 *
 * 3. INDIKATOR LED & BUZZER (Mirip Firmware TX Custom V930):
 *    - LED Merah (DISARM / Status Multiprotocol) -> Pin D13 (LED L onboard / ekst.)
 *    - LED Hijau/Biru (ARMED / Aktif CustomRX)   -> Pin D6 (+ Resistor 220-330 Ohm ke GND)
 *    - Buzzer Alarm & Notifikasi                 -> Pin D8 (Kaki + ke D8, kaki - ke GND)
 *      (Pilih tipe Buzzer Pasif/Aktif lewat variabel PAKE_BUZZER_PASIF di bawah)
 *
 * ============================================================================
 * PANDUAN PENGGUNAAN DI REMOTE MICROZONE MC6C
 * ============================================================================
 * A. MEMILIH PROTOKOL SAAT MENYALAKAN REMOTE (Tahan stik dulu, baru ON):
 *    - Custom RX Universal : Stik Kiri ke KIRI + Stik Kanan ke KANAN (Menjauh)
 *    - Eachine E010 / NH010: Rudder Kanan + Aileron Kanan + Elevator Bawah
 *    - Protokol terakhir   : Biarkan semua stik di tengah (otomatis dari EEPROM)
 *    (Pastikan saklar D/R di posisi penuh 100% & MIX di NOR saat memilih protokol)
 *
 * B. CARA BINDING RX CUSTOM BARU (10 Detik di Channel 10, Alamat "BIND0"):
 *    1. Pastikan protokol aktif sudah di Custom RX (atau tahan stik menjauh).
 *    2. Naikkan KEDUA saklar bahu CH.5 = ATAS dan CH.6 = ATAS sebelum power ON.
 *    3. Nyalakan RX (jumper D2 RX ke GND) lalu nyalakan remote Microzone.
 *    4. LED Merah & Hijau berkedip bergantian + Buzzer berbunyi selama 10 detik.
 *    5. Setelah selesai, turunkan CH.5 ke BAWAH untuk membuka kunci pengaman.
 *
 * C. FUNGSI SAKLAR MICROZONE SAAT MODE CUSTOM RX:
 *    [SAKLAR CH.5 - 3 Posisi (Arming & Tipe Motor)]:
 *    - BAWAH  : DISARM (Motor terkunci aman, LED Merah nyala)
 *    - TENGAH : ARM -> Mode ESC Brushless (motorType = 0, LED Hijau nyala)
 *    - ATAS   : ARM -> Mode MOSFET Brushed (motorType = 1, LED Hijau nyala)
 *    *Keamanan: Arming ditolak jika gas belum nol! Ganti ESC/MOSFET wajib Disarm dulu.
 *
 *    [SAKLAR CH.6 - 3 Posisi (Mode Kendaraan - hanya bisa diganti saat DISARM)]:
 *    - BAWAH  : Mode 1 (Pesawat Normal; aktifkan saklar MIX+ELEVON di bodi
 *               Microzone untuk otomatis menjadi pesawat Jet/Delta Wing!)
 *    - TENGAH : Mode 2 (Pesawat Diferensial 2 Motor - wajib MIX = NOR)
 *    - ATAS   : Mode 4 (Mobil RC Propo + AUX1 Lampu + AUX2 Musik - wajib MIX = NOR)
 *
 *    [SAKLAR PANEL BAWAH MICROZONE]:
 *    - SERVO REVERSER (AIL/ELE/THR/RUD) : Langsung membalik arah sinyal PPM.
 *    - MIX / V-TAIL / ELEVON            : Gunakan di Mode 1 untuk sayap Delta.
 * ============================================================================
 */

#include <Arduino.h>
#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

// ============================================================================
// 0. PANEL KONFIGURASI MASTER (UBAH DI SINI SEBELUM UPLOAD)
// ============================================================================
// Setel ke 1 saat debugging di meja (Serial Monitor 115200 bps aktif + Cek NRF).
// WAJIB setel ke 0 saat sudah final/terbang agar hemat Flash/RAM & bebas lag!
#define AKTIFKAN_SERIAL 0

#if AKTIFKAN_SERIAL == 0
  // Menghapus seluruh perintah Serial saat kompilasi (0 byte overhead)
  #define Serial if(false)Serial
#endif

// Konfigurasi Tipe Buzzer pada Pin D8:
// true  = Buzzer Pasif (menggunakan fungsi tone() di frekuensi 2500 Hz)
// false = Buzzer Aktif (2 kaki standar, cukup diberi tegangan HIGH/LOW)
const bool PAKE_BUZZER_PASIF = true;
const int  FREKUENSI_BUZZER  = 2500;

// Pembalik Arah Saklar CH.5 dan CH.6 (Ubah ke true jika posisi Bawah/Atas terbalik)
#define CH5_REVERSE false
#define CH6_REVERSE false

// ============================================================================
// 1. DEFINISI PIN HARDWARE ARDUINO PRO MINI / NANO
// ============================================================================
#define PPM_pin        2  // Input sinyal PPM dari remote Microzone (Pin D2)

// Pin Komunikasi Software SPI ke modul nRF24L01
#define MOSI_pin       3  // MOSI -> Pin D3
#define SCK_pin        4  // SCK  -> Pin D4
#define CE_pin         5  // CE   -> Pin D5
#define MISO_pin       A0 // MISO -> Pin A0
#define CS_pin         A1 // CSN  -> Pin A1

// Pin Indikator LED & Buzzer
#define ledPin         13 // LED Merah / LED L onboard (DISARM & status multiprotocol)
#define PIN_LED_MERAH  13 // Alias untuk kejelasan di protokol CustomRX
#define PIN_LED_HIJAU  6  // LED Hijau / Biru eksternal (ARMED / CustomRX aktif)
#define PIN_BUZZER     8  // Output Buzzer (Aktif / Pasif sesuai konfigurasi di atas)

// Kendali Cepat Output SPI Tingkat Register Port Mikrokontroler
#define MOSI_on  PORTD |= _BV(3)   // PD3 HIGH
#define MOSI_off PORTD &= ~_BV(3)  // PD3 LOW
#define SCK_on   PORTD |= _BV(4)   // PD4 HIGH
#define SCK_off  PORTD &= ~_BV(4)  // PD4 LOW
#define CE_on    PORTD |= _BV(5)   // PD5 HIGH
#define CE_off   PORTD &= ~_BV(5)  // PD5 LOW
#define CS_on    PORTC |= _BV(1)   // PC1 HIGH
#define CS_off   PORTC &= ~_BV(1)  // PC1 LOW
#define MISO_on  (PINC & _BV(0))   // Baca PC0 (A0)

// Daya Pancar Radio NRF24L01 (80mW / PA_MAX untuk jangkauan maksimal)
#define RF_POWER TX_POWER_80mW

// ============================================================================
// 2. PENGATURAN ALIRAN SINYAL PPM (STANDAR MICROZONE MC6C = AETR)
// ============================================================================
#define CHANNELS 12 // Kapasitas array channel PPM (Microzone memakai CH1-CH6)
enum chan_order {
    AILERON,  // CH1 Microzone: Miring Kiri / Kanan (Roll)
    ELEVATOR, // CH2 Microzone: Maju / Mundur (Pitch)
    THROTTLE, // CH3 Microzone: Gas / Daya Angkat (Stik Kiri Vertikal)
    RUDDER,   // CH4 Microzone: Putar Haluan Kiri / Kanan (Yaw)
    AUX1,     // CH5 Microzone: Saklar Bahu Kiri 3-Posisi (Arm/Disarm & MotorType di CustomRX)
    AUX2,     // CH6 Microzone: Saklar Bahu Kanan 3-Posisi (Mode Kendaraan di CustomRX / Flip di E010)
    AUX3,     // CH7 (Cadangan)
    AUX4,     // CH8 (Cadangan)
    AUX5,     // CH9 (Cadangan)
    AUX6,     // CH10 (Cadangan)
    AUX7,     // CH11 (Cadangan)
    AUX8,     // CH12 (Reset / Rebind Multiprotocol)
};

// Batas Nilai Pulsa PPM (dalam satuan mikrodetik / us)
#define PPM_MIN           1000 // Nilai minimum stik
#define PPM_SAFE_THROTTLE 1050 // Batas aman gas bawah saat menyalakan remote
#define PPM_MID           1500 // Nilai tengah netral stik
#define PPM_MAX           2000 // Nilai maksimum stik
#define PPM_MIN_COMMAND   1300 // Ambang bawah deteksi perintah stik/saklar
#define PPM_MAX_COMMAND   1700 // Ambang atas deteksi perintah stik/saklar
#define GET_FLAG(ch, mask)     (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// ============================================================================
// 3. DAFTAR PROTOKOL YANG DIDUKUNG FIRMWARE
// ============================================================================
enum {
    PROTO_V2X2 = 0,          // WLToys V2x2, JXD JD38x, JJRC H6C
    PROTO_CG023,             // Eachine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,         // Cheerson CX-10 PCB biru / merah baru
    PROTO_CX10_GREEN,        // Cheerson CX-10 PCB hijau lama
    PROTO_H7,                // Eachine H7, MoonTop M99xx
    PROTO_BAYANG,            // Eachine H8 mini, BayangToys X6/X7/X9
    PROTO_SYMAX5C1,          // Syma X5C-1, X11, X12
    PROTO_YD829,             // Attop YD-829, YD-822
    PROTO_H8_3D,             // Eachine H8 mini 3D, JJRC H20/H22
    PROTO_MJX,               // MJX X600
    PROTO_SYMAXOLD,          // Syma X5C lama, X2
    PROTO_HISKY,             // HiSky RX, WLToys V933/V944/V955
    PROTO_KN,                // WLToys V930/V977/V988
    PROTO_YD717,             // Cheerson CX-10 merah lama, JXD389
    PROTO_FQ777124,          // FQ777-124 Pocket Drone
    PROTO_E010,              // Eachine E010, NiHui NH-010, JJRC H36 mini
    PROTO_BAYANG_SILVERWARE, // Bayang Silverware dengan telemetri FrSky
    PROTO_CUSTOM_RX,         // [BARU] Protokol Custom RX Universal Hybrid (FHSS 4-Ch)
    PROTO_END
};

// Alamat Penyimpanan EEPROM Bawaan Multiprotocol (Byte 0 s/d 4)
// Catatan: CustomRX menggunakan alamat EEPROM terpisah (Byte 20 s/d 26) agar tidak bentrok!
enum {
    ee_PROTOCOL_ID = 0, // Menyimpan ID protokol terakhir yang dipakai
    ee_TXID0,           // Byte 1 ID Transmitter multiprotocol
    ee_TXID1,           // Byte 2 ID Transmitter multiprotocol
    ee_TXID2,           // Byte 3 ID Transmitter multiprotocol
    ee_TXID3            // Byte 4 ID Transmitter multiprotocol
};

// Struktur Data Telemetri Bawaan Multiprotocol (FrSky / Silverware)
struct {
    uint16_t volt1;
    uint16_t rssi;
    uint8_t  updated;
    uint32_t lastUpdate;
} telemetry_data;

// Variabel Global Sistem Multiprotocol
uint8_t transmitterID[4];             // ID unik pemancar untuk drone mainan
uint8_t current_protocol;             // Protokol yang sedang aktif
static volatile bool ppm_ok = false;  // Penanda frame PPM valid dari interrupt
uint8_t packet[32];                   // Buffer paket data radio
static bool reset = true;             // Pemicu inisialisasi saat baru menyala
volatile uint16_t Servo_data[12];     // Buffer mentah hasil ukur Timer1 pada ISR
static uint16_t ppm[12] = {
    PPM_MID, PPM_MID, PPM_MIN, PPM_MID, // AIL=1500, ELE=1500, THR=1000, RUD=1500
    PPM_MIN, PPM_MIN,                   // CH5=1000 (DISARM), CH6=1000 (Mode 1)
    PPM_MID, PPM_MID, PPM_MID, PPM_MID, PPM_MID, PPM_MID
};

// ============================================================================
// 4. FUNGSI PEMBANTU BUZZER & DIAGNOSTIK REGISTER NRF24L01
// ============================================================================
// Mengendalikan buzzer fisik secara seragam (mendukung Buzzer Pasif maupun Aktif)
void nyalakanBuzzer(bool nyala) {
    if (nyala) {
        if (PAKE_BUZZER_PASIF) tone(PIN_BUZZER, FREKUENSI_BUZZER);
        else digitalWrite(PIN_BUZZER, HIGH);
    } else {
        if (PAKE_BUZZER_PASIF) noTone(PIN_BUZZER);
        else digitalWrite(PIN_BUZZER, LOW);
    }
}

// Bunyi beep singkat blocking (hanya dipakai saat booting/setup/binding)
void beepBlocking(unsigned int duration) {
    nyalakanBuzzer(true);
    delay(duration);
    nyalakanBuzzer(false);
}

// Fungsi Diagnostik Hardware NRF24L01 (Aktif jika AKTIFKAN_SERIAL == 1)
// Memeriksa seluruh register inti NRF24 lewat Software SPI untuk memastikan modul sehat
bool cekRegisterNRF24(const char* label) {
    uint8_t regConfig  = NRF24L01_ReadReg(NRF24L01_00_CONFIG);
    uint8_t regEnAA    = NRF24L01_ReadReg(NRF24L01_01_EN_AA);
    uint8_t regSetupAW = NRF24L01_ReadReg(NRF24L01_03_SETUP_AW);
    uint8_t regRetr    = NRF24L01_ReadReg(NRF24L01_04_SETUP_RETR);
    uint8_t regRfCh    = NRF24L01_ReadReg(NRF24L01_05_RF_CH);
    uint8_t regRfSetup = NRF24L01_ReadReg(NRF24L01_06_RF_SETUP);
    uint8_t regStatus  = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    uint8_t regDynpd   = NRF24L01_ReadReg(NRF24L01_1C_DYNPD);
    uint8_t regFeature = NRF24L01_ReadReg(NRF24L01_1D_FEATURE);

    Serial.println(F("========================================"));
    Serial.print(F("[NRF CHECK] ")); Serial.println(label);
    Serial.print(F("  CONFIG   (0x00) : 0x")); Serial.println(regConfig, HEX);
    Serial.print(F("  EN_AA    (0x01) : 0x")); Serial.println(regEnAA, HEX);
    Serial.print(F("  SETUP_AW (0x03) : 0x")); Serial.println(regSetupAW, HEX);
    Serial.print(F("  RETR     (0x04) : 0x")); Serial.println(regRetr, HEX);
    Serial.print(F("  RF_CH    (0x05) : "));   Serial.println(regRfCh);
    Serial.print(F("  RF_SETUP (0x06) : 0x")); Serial.println(regRfSetup, HEX);
    Serial.print(F("  STATUS   (0x07) : 0x")); Serial.println(regStatus, HEX);
    Serial.print(F("  DYNPD    (0x1C) : 0x")); Serial.println(regDynpd, HEX);
    Serial.print(F("  FEATURE  (0x1D) : 0x")); Serial.println(regFeature, HEX);

    // Jika STATUS atau CONFIG terbaca 0x00 atau 0xFF, berarti SPI putus atau NRF mati/kurang daya
    bool nrfSehat = !(regStatus == 0x00 || regStatus == 0xFF || regConfig == 0xFF);
    if (!nrfSehat) {
        Serial.println(F("[FATAL] NRF24L01 TIDAK MERESPON SPI! Cek kabel D3/D4/D5/A0/A1 & power 3.3V!"));
    } else {
        Serial.println(F("[OK] Modul NRF24L01 terdeteksi normal melalui Software SPI."));
    }
    Serial.println(F("========================================"));
    return nrfSehat;
}

// Deklarasi fungsi protokol CustomRX (berada di file CustomRX.ino)
void CustomRX_init();
uint32_t process_CustomRX();

// ============================================================================
// 5. FUNGSI UTAMA: SETUP (DIJALANKAN 1x SAAT REMOTE DINYALAKAN)
// ============================================================================
void setup() {
    // Bibit bilangan acak dari noise pin analog kosong A4 & A5
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));

    // Inisialisasi pin LED Merah (D13), LED Hijau (D6), dan Buzzer (D8)
    pinMode(PIN_LED_MERAH, OUTPUT);
    pinMode(PIN_LED_HIJAU, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED_MERAH, HIGH); // Awal booting: Merah nyala (status aman/disarm)
    digitalWrite(PIN_LED_HIJAU, LOW);  // Hijau mati
    nyalakanBuzzer(false);

    // Inisialisasi pin input PPM dan pin Software SPI NRF24L01
    pinMode(PPM_pin, INPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    // Pengaturan Serial Debugging vs Telemetri FrSky (agar UART tidak bentrok)
    #if AKTIFKAN_SERIAL == 1
        Serial.begin(115200);
        delay(100);
        Serial.println(F("\n[BOOT] TX MULTIPROTOCOL + CUSTOM RX UNIVERSAL SIAP"));
        Serial.flush();
    #else
        frskyInit(); // Aktifkan telemetri FrSky bawaan hanya saat Serial Debug dimatikan
    #endif

    // Pengujian koneksi awal NRF24L01 sebelum menunggu sinyal PPM
    NRF24L01_Reset();
    NRF24L01_Initialize();
    if (!cekRegisterNRF24("Inisialisasi Awal (Boot)")) {
        // Bunyikan 3x bip peringatan jika NRF bermasalah saat booting
        for (uint8_t i = 0; i < 3; i++) {
            beepBlocking(120);
            delay(100);
        }
    }

    // Aktifkan Interrupt pembaca sinyal PPM pada Pin D2
    attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);
    TCCR1A = 0;             // Reset Timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  // Prescaler 8 -> 0.5 us per tick @ 16MHz (1 us @ 8MHz)

    set_txid(false); // Muat ID Transmitter multiprotocol dari EEPROM
}

// ============================================================================
// 6. FUNGSI UTAMA: LOOP (EKSEKUSI REAL-TIME PENGIRIMAN SINYAL RADIO)
// ============================================================================
void loop() {
    uint32_t timeout = 0;

    // Jalankan pemilihan protokol & inisialisasi radio saat booting atau tombol rebind (AUX8)
    if (reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }

    telemetry_data.updated = 0;

    // Eksekusi pengiriman paket radio sesuai protokol yang sedang aktif
    switch (current_protocol) {
        case PROTO_CG023:
        case PROTO_YD829:
            timeout = process_CG023();
            break;
        case PROTO_V2X2:
            timeout = process_V2x2();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            timeout = process_CX10();
            break;
        case PROTO_H7:
            timeout = process_H7();
            break;
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            timeout = process_Bayang();
            break;
        case PROTO_SYMAX5C1:
        case PROTO_SYMAXOLD:
            timeout = process_SymaX();
            break;
        case PROTO_H8_3D:
            timeout = process_H8_3D();
            break;
        case PROTO_MJX:
        case PROTO_E010:
            timeout = process_MJX();
            break;
        case PROTO_HISKY:
            timeout = process_HiSky();
            break;
        case PROTO_KN:
            timeout = process_KN();
            break;
        case PROTO_YD717:
            timeout = process_YD717();
            break;
        case PROTO_FQ777124:
            timeout = process_FQ777124();
            break;
        case PROTO_CUSTOM_RX:
            timeout = process_CustomRX(); // Jalankan mesin FHSS Custom RX Universal
            break;
    }

    // Perbarui nilai array ppm[] secara aman dari buffer interrupt
    update_ppm();

    // Tunggu sampai jadwal paket berikutnya tiba
    while (micros() < timeout) {
        #if AKTIFKAN_SERIAL == 0
            if (telemetry_data.updated) {
                frskyUpdate();
            }
        #endif
    }
    telemetry_data.updated = 0;
}

// ============================================================================
// 7. MANAJEMEN TX ID & PEMILIHAN PROTOKOL VIA KOMBINASI STIK
// ============================================================================
void set_txid(bool renew) {
    uint8_t i;
    for (i = 0; i < 4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0 + i);
    if (renew || (transmitterID[0] == 0xFF && transmitterID[1] == 0xFF)) {
        for (i = 0; i < 4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0 + i, transmitterID[i]);
        }
    }
}

void selectProtocol() {
    // Tunggu 10 frame sinyal PPM valid (~200ms) agar bacaan stik stabil
    Serial.println(F("[PPM] Menunggu sinyal PPM valid dari remote Microzone..."));
    ppm_ok = false;
    uint8_t count = 10;
    while (count) {
        while (!ppm_ok) {} // Menunggu interrupt PPM
        update_ppm();
        if (ppm[AUX8] < PPM_MAX_COMMAND)
            count--;
        ppm_ok = false;
    }

    Serial.print(F("[PPM] OK! AIL:")); Serial.print(ppm[AILERON]);
    Serial.print(F(" ELE:"));        Serial.print(ppm[ELEVATOR]);
    Serial.print(F(" THR:"));        Serial.print(ppm[THROTTLE]);
    Serial.print(F(" RUD:"));        Serial.print(ppm[RUDDER]);
    Serial.print(F(" CH5:"));        Serial.print(ppm[AUX1]);
    Serial.print(F(" CH6:"));        Serial.println(ppm[AUX2]);

    // ===== DETEKSI KOMBINASI STIK SAAT POWER-ON =====

    // [PRIORITAS 1] Kedua stik didorong MENJAUH (Rudder Kiri + Aileron Kanan) -> Custom RX Universal
    // WAJIB diletakkan di atas cek Rudder Kiri tunggal agar tidak tertangkap oleh set_txid(true)!
    if (ppm[RUDDER] < PPM_MIN_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_CUSTOM_RX;

    // Rudder Kiri + Aileron Kiri -> Bayang Silverware
    else if (ppm[RUDDER] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_BAYANG_SILVERWARE;

    // Rudder Kiri saja -> Acak ulang Transmitter ID multiprotocol
    else if (ppm[RUDDER] < PPM_MIN_COMMAND)
        set_txid(true);

    // Rudder Kanan + Aileron Kanan + Elevator Bawah -> Eachine E010 / NH-010 / H36
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_E010;

    // Rudder Kanan + Aileron Kanan + Elevator Atas -> FQ777-124
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_FQ777124;

    // Rudder Kanan + Aileron Kiri + Elevator Atas -> YD717 / CX-10 merah lama
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD717;

    // Rudder Kanan + Aileron Kiri + Elevator Bawah -> KN (WLToys V930/V977)
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_KN;

    // Rudder Kanan + Elevator Bawah -> HiSky
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_HISKY;

    // Rudder Kanan + Elevator Atas -> Syma X5C lama
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_SYMAXOLD;

    // Rudder Kanan + Aileron Kanan -> MJX X600
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_MJX;

    // Rudder Kanan + Aileron Kiri -> Eachine H8 3D / H20
    else if (ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_H8_3D;

    // Elevator Bawah + Aileron Kanan -> YD-829
    else if (ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD829;

    // Elevator Bawah + Aileron Kiri -> Syma X5C-1
    else if (ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_SYMAX5C1;

    // Elevator Atas + Aileron Kanan -> Bayang / H8 mini
    else if (ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_BAYANG;

    // Elevator Atas + Aileron Kiri -> Eachine H7
    else if (ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_H7;

    // Elevator Atas saja -> V2x2
    else if (ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_V2X2;

    // Elevator Bawah saja -> CG023
    else if (ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_CG023;

    // Aileron Kanan saja -> CX-10 PCB Biru
    else if (ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_CX10_BLUE;

    // Aileron Kiri saja -> CX-10 PCB Hijau
    else if (ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_CX10_GREEN;

    // Stik Netral -> Muat protokol terakhir yang tersimpan di EEPROM
    else
        current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID), 0, PROTO_END - 1);

    // Simpan protokol aktif ke EEPROM
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);

    Serial.print(F("[PROTO] Protokol Terpilih (ID "));
    Serial.print(current_protocol);
    if (current_protocol == PROTO_CUSTOM_RX) Serial.println(F("): CUSTOM RX UNIVERSAL"));
    else if (current_protocol == PROTO_E010) Serial.println(F("): EACHINE E010"));
    else Serial.println(F("): MULTIPROTOCOL DRONE"));

    // Pengaman Throttle Awal: Untuk drone mainan, tunggu gas ditarik ke bawah dulu.
    // (Untuk CustomRX, pengaman gas & saklar CH.5 dikelola langsung di CustomRX.ino
    // agar mode binding via saklar CH5+CH6 tetap bisa dijalankan dengan mulus)
    if (current_protocol != PROTO_CUSTOM_RX) {
        while (ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
            digitalWrite(PIN_LED_MERAH, LOW);
            delay(100);
            update_ppm();
        }
        digitalWrite(PIN_LED_MERAH, HIGH);
        digitalWrite(PIN_LED_HIJAU, LOW);
        beepBlocking(120); // 1x bip pendek tanda mode drone mainan siap
    }
}

// Memanggil fungsi inisialisasi & binding protokol terpilih
void init_protocol() {
    switch (current_protocol) {
        case PROTO_CG023:
        case PROTO_YD829:
            CG023_init();
            CG023_bind();
            break;
        case PROTO_V2X2:
            V2x2_init();
            V2x2_bind();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            CX10_init();
            CX10_bind();
            break;
        case PROTO_H7:
            H7_init();
            H7_bind();
            break;
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            Bayang_init();
            Bayang_bind();
            break;
        case PROTO_SYMAX5C1:
        case PROTO_SYMAXOLD:
            Symax_init();
            break;
        case PROTO_H8_3D:
            H8_3D_init();
            H8_3D_bind();
            break;
        case PROTO_MJX:
        case PROTO_E010:
            MJX_init();
            MJX_bind();
            break;
        case PROTO_HISKY:
            HiSky_init();
            break;
        case PROTO_KN:
            kn_start_tx(true);
            break;
        case PROTO_YD717:
            YD717_init();
            break;
        case PROTO_FQ777124:
            FQ777124_init();
            FQ777124_bind();
            break;
        case PROTO_CUSTOM_RX:
            CustomRX_init(); // Inisialisasi Custom RX Universal Hybrid (FHSS + Auto-ACK)
            break;
    }
}

// ============================================================================
// 8. PEMBACAAN SINYAL PPM (ATOMIC COPY & INTERRUPT SERVICE ROUTINE)
// ============================================================================
void update_ppm() {
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }
}

void ISR_ppm() {
    #if F_CPU == 16000000
        #define PPM_SCALE 1L // 16 MHz (0.5 us per tick)
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L // 8 MHz (1.0 us per tick)
    #else
        #error "Hanya mendukung clock 8 MHz atau 16 MHz!"
    #endif

    static unsigned int  pulse;
    static unsigned long counterPPM;
    static byte          chan;

    counterPPM = TCNT1;
    TCNT1      = 0;
    ppm_ok     = false;

    if (counterPPM < (510 << PPM_SCALE)) {          // Pulsa pemisah (< 510 us)
        pulse = counterPPM;
    } else if (counterPPM > (1910 << PPM_SCALE)) {  // Jeda akhir frame (> 1910 us)
        chan = 0;
    } else {                                        // Durasi channel (510 - 2420 us)
        if (chan < CHANNELS) {
            Servo_data[chan] = constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if (chan == 3)
                ppm_ok = true; // 4 channel utama (AETR) sudah terbaca lengkap
        }
        chan++;
    }
}