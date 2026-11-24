/*
 * ============================================================================
 * MODUL PROTOKOL: CUSTOM RX UNIVERSAL HYBRID (FHSS 4-CH + AUTO-ACK TELEMETRI)
 * ============================================================================
 * Ditujukan Untuk : Receiver (RX) Custom Universal Hybrid buatan sendiri
 *                   (Pesawat Normal/Elevon, Diferensial 2 Motor, Mobil RC Propo)
 * Target Remote   : Microzone MC6C (1 Profil Universal + CH5 3-Pos + CH6 2-Pos)
 *
 * ============================================================================
 * PANDUAN SINGKAT KENDALI DI MICROZONE MC6C (MODE CUSTOM RX)
 * ============================================================================
 * 1. MASUK KE PROTOKOL CUSTOM RX:
 *    - Tahan kedua stik MENJAUH (Stik Kiri ke KIRI + Stik Kanan ke KANAN) saat ON.
 *    - Pemakaian berikutnya cukup nyalakan remote normal (otomatis dari EEPROM).
 *
 * 2. BINDING RX BARU (10 Detik di Channel 10, Alamat "BIND0"):
 *    - Naikkan KEDUA saklar CH.5 = ATAS (2000) dan CH.6 = ATAS (2000) sebelum ON.
 *    - Setelah selesai, turunkan CH.5 & CH.6 ke BAWAH (1000) sebelum dipakai.
 *    - Semua RX Custom cukup di-bind 1x seumur hidup ke remote MC6C ini.
 *
 * 3. SAKLAR CH.5 (3-Posisi: Arming & Tipe Motor + Jeda Anti-Salah-Lewat 250ms):
 *    - BAWAH  (1000) : DISARM instan (Motor terkunci aman, LED Merah D13 nyala).
 *                      Tipe motor terakhir tetap dipertahankan agar Brushed diam
 *                      total (0V) dan ESC Brushless tetap tenang (1000us).
 *    - TENGAH (1500) : ARM -> Mode MOSFET Brushed (motorType = 1, LED Hijau D6).
 *    - ATAS   (2000) : ARM -> Mode ESC Brushless  (motorType = 0, LED Hijau D6).
 *    *Keamanan & Keawetan EEPROM:
 *     - Arming ditolak jika gas belum 0!
 *     - Jeda penstabil 250ms mencegah lintasan sesaat di posisi Tengah saat
 *       menggeser cepat Atas <-> Bawah mengubah tipe motor.
 *     - Saat gas masih 0 (di darat), menggeser Tengah <-> Atas langsung berpindah
 *       MOSFET <-> ESC setelah diam 250ms. Saat gas naik (terbang), terkunci aman!
 *     - Memakai EEPROM.update() -> 0 siklus tulis saat Arm/Disarm berulang!
 *
 * 4. SAKLAR CH.6 (2-Posisi Taktil: Ganti Mode Berurutan 1 -> 2 -> 3 -> 1):
 *    - Hanya berfungsi saat DISARM (CH.5 di Bawah).
 *    - Geser CH.6 dari BAWAH (1000) ke ATAS (2000) -> Mode maju 1 langkah &
 *      tersimpan di EEPROM. Wajib turunkan dulu ke BAWAH untuk pindah lagi.
 *    - Urutan Mode TX (Otomatis diterjemahkan ke auxFlag RX tanpa ubah kode RX!):
 *      * Mode 1 (1x Bip + 1x Kedip Hijau) : Pesawat Normal -> kirim auxFlag = 1
 *        (Rudder, Elevator, Aileron murni -> bisa langsung pakai MIX+ELEVON MC6C!)
 *      * Mode 2 (2x Bip + 2x Kedip Hijau) : Diferensial 2 Motor -> kirim auxFlag = 2
 *        (Pastikan saklar MIX di bodi MC6C posisi NOR)
 *      * Mode 3 (3x Bip + 3x Kedip Hijau) : Mobil RC Propo -> kirim auxFlag = 4
 *        (Pastikan saklar MIX di bodi MC6C posisi NOR)
 *
 * ============================================================================
 * KOMPATIBILITAS RADIO & ALOKASI EEPROM
 * ============================================================================
 * - Radio RF24 Match : 1 Mbps, CRC 16-bit (0x0E), Auto-ACK (0x3F), Retries 500us x 3
 *                      (0x13), Dynamic Payload + ACK Payload Telemetri 2 Byte (0x06).
 * - Rumus FHSS 4-Ch  : 20 + ((seed - 20 + (i * 23)) % 80), interval kirim 30 ms.
 * - Alokasi EEPROM   : Terpisah dari Byte 0-4 milik Multiprotocol:
 *                      Byte 20    = Segel validasi (0xAC)
 *                      Byte 21    = Bibit channel operasional (20..94)
 *                      Byte 22-26 = 5 Byte alamat pipa rahasia (activePipe)
 *                      Byte 27    = Urutan mode terakhir (1, 2, atau 3)
 *                      Byte 28    = Tipe motor terakhir (1=MOSFET, 0=ESC)
 * ============================================================================
 */

// ============================================================================
// 1. KONSTANTA & STRUKTUR PAKET RADIO (WAJIB 100% IDENTIK DENGAN FIRMWARE RX)
// ============================================================================
#define CUSTOM_BIND_CH       10       // Channel darurat khusus proses Binding
#define CUSTOM_TX_INTERVAL   30000UL  // Interval kirim 30.000 us = 30 ms (33 Hz)
#define CUSTOM_FHSS_LEN      4        // Jumlah saluran lompatan frekuensi FHSS
#define CH5_MOTOR_SETTLE_MS  250UL    // Jeda konfirmasi 250ms anti-lewat-tengah pada CH.5

// Alamat EEPROM khusus CustomRX (tidak bentrok dengan Byte 0-4 Multiprotocol)
#define EE_CUSTOM_MAGIC      20       // Alamat segel validasi (0xAC = sudah terisi)
#define EE_CUSTOM_CH         21       // Alamat bibit channel (20..94)
#define EE_CUSTOM_PIPE       22       // Alamat 5 byte pipa (byte 22, 23, 24, 25, 26)
#define EE_CUSTOM_MODE       27       // Alamat penyimpan mode terakhir (1, 2, atau 3)
#define EE_CUSTOM_MOTOR      28       // Alamat penyimpan tipe motor terakhir (1=MOSFET, 0=ESC)

// Alamat pintu darurat binding (5 byte, sama persis dengan "BIND0" di RX)
static const uint8_t CUSTOM_BIND_ADDR[5] = {'B', 'I', 'N', 'D', '0'};

// Struktur paket Binding (Total: 7 Byte)
struct CustomBindPayload {
    uint8_t pipeAddr[5]; // 5 huruf kapital acak sebagai alamat unik RX
    uint8_t opChannel;   // 1 angka bibit channel (20..94)
    uint8_t checkSum;    // Verifikasi XOR: opChannel ^ pipeAddr[0] ^ pipeAddr[4]
};

// Struktur paket Kendali Utama (Total: 7 Byte - WAJIB urut & identik dengan RX)
struct CustomControlData {
    uint8_t throttle;  // Gas (0 s/d 255)
    int8_t  rudder;    // Rudder (-127 s/d +127)
    int8_t  elevator;  // Elevator (-127 s/d +127)
    int8_t  aileron;   // Aileron (-127 s/d +127)
    uint8_t isArmed;   // 0 = Disarm (Kunci Motor), 1 = Armed (Hidup)
    uint8_t auxFlag;   // Mode RX: 1=Pesawat Normal/Elevon, 2=Diferensial, 4=Mobil RC
    uint8_t motorType; // Penggerak: 0=ESC Brushless, 1=MOSFET Brushed
};

// Struktur paket Telemetri Balasan dari RX via ACK Payload (Total: 2 Byte)
struct CustomTelemetryData {
    uint8_t batteryStatus;  // 0 = Normal, 1 = Baterai 1S RX Lemah (<= 3.5V)
    uint8_t failsafeActive; // 0 = Terkendali, 1 = RX dalam mode failsafe
};

// ============================================================================
// 2. VARIABEL INTERNAL PROTOKOL CUSTOM RX
// ============================================================================
static CustomControlData   customTxData;            // Paket kendali yang dikirim ke RX
static CustomTelemetryData customRxTelem;           // Paket telemetri balasan dari RX

static uint8_t customPipe[5];                       // 5 byte alamat operasional aktif
static uint8_t customSeedChannel = 76;              // Bibit channel operasional (20..94)
static uint8_t customFhssChannels[CUSTOM_FHSS_LEN]; // Daftar 4 channel lompatan FHSS
static uint8_t customHopIndex = 0;                  // Indeks lompatan aktif (0..3)

static bool          customIsArmed         = false; // Status keamanan motor
static bool          customMustDisarmFirst = true;  // Kunci pengaman saat booting/pasca-bind
static bool          customCh6WasHigh      = true;  // Pengunci saklar taktil CH.6 (anti-loncat)
static uint8_t       customModeIndex       = 1;     // Urutan mode TX: 1, 2, atau 3
static uint8_t       customActiveMotorType = 1;     // Tipe motor aktif: 1=MOSFET (Tengah), 0=ESC (Atas)
static uint8_t       customPendingMotor    = 255;   // Penampung transisi CH.5 (255 = tidak ada)
static unsigned long customMotorTimer      = 0;     // Pewaktu jeda 250ms penstabil CH.5
static unsigned long customLastAckTime     = 0;     // Waktu terakhir menerima ACK dari RX
static unsigned long customLastAlarmTime   = 0;     // Pewaktu alarm baterai 1S RX lemah
static unsigned long customLastDebugPrint  = 0;     // Pewaktu cetak log Serial debugging

// Variabel Mesin Buzzer & Kedip LED Non-Blocking (Mendukung 1x, 2x, 3x bip + kedip)
static uint8_t       customBeepRemaining = 0;     // Sisa jumlah bunyi bip yang harus dimainkan
static bool          customBeepIsOn      = false; // Status sedang berbunyi (true) atau jeda diam (false)
static bool          customBeepFlashGrn  = false; // true = ikut kedipkan LED Hijau (saat ganti mode)
static unsigned long customBeepTimer     = 0;     // Stopwatch durasi bip / jeda
static unsigned int  customBeepOnMs      = 100;   // Durasi menyala tiap bip (ms)
static unsigned int  customBeepOffMs     = 100;   // Durasi jeda antar bip (ms)

// ============================================================================
// 3. FUNGSI PEMBANTU: PEMETAAN MODE & AUDIO-VISUAL NON-BLOCKING
// ============================================================================
// Menerjemahkan urutan mode TX (1, 2, 3) ke nilai auxFlag RX (1, 2, 4)
static uint8_t customGetRxAuxFlag(uint8_t modeIdx) {
    if (modeIdx == 3) return 4; // Urutan 3 di Microzone -> Mode 4 (Mobil RC) di RX
    if (modeIdx == 2) return 2; // Urutan 2 di Microzone -> Mode 2 (Diferensial) di RX
    return 1;                   // Urutan 1 di Microzone -> Mode 1 (Pesawat Normal / Elevon) di RX
}

// Memicu pola N kali bip secara non-blocking (aman, tidak menghambat FHSS 30ms)
static void customBeepN(uint8_t count, unsigned int onMs, unsigned int offMs, bool flashGreen) {
    if (count == 0) return;
    customBeepRemaining = count;
    customBeepOnMs      = onMs;
    customBeepOffMs     = offMs;
    customBeepFlashGrn  = flashGreen;
    customBeepIsOn      = true;
    customBeepTimer     = millis();
    nyalakanBuzzer(true);
}

// Memproses Buzzer, LED Merah (D13), dan LED Hijau (D6) setiap putaran loop
static void customHandleAudioVisual() {
    unsigned long now = millis();

    if (customBeepRemaining > 0) {
        if (customBeepIsOn) {
            if (now - customBeepTimer >= customBeepOnMs) {
                nyalakanBuzzer(false);
                customBeepIsOn = false;
                customBeepRemaining--;
                customBeepTimer = now;
            }
        } else {
            if (now - customBeepTimer >= customBeepOffMs) {
                nyalakanBuzzer(true);
                customBeepIsOn  = true;
                customBeepTimer = now;
            }
        }
    } else {
        customBeepIsOn     = false;
        customBeepFlashGrn = false;
    }

    // Kendali Visual LED Merah (D13) & LED Hijau (D6):
    if (customIsArmed) {
        // ARMED: Hijau nyala solid; saat buzzer bunyi, Merah berkedip bergantian
        digitalWrite(PIN_LED_HIJAU, customBeepIsOn ? LOW : HIGH);
        digitalWrite(PIN_LED_MERAH, customBeepIsOn ? HIGH : LOW);
    } else {
        // DISARMED:
        if (customBeepFlashGrn) {
            // Saat ganti mode via CH.6: Kedipkan LED Hijau persis sesuai jumlah bip (1x / 2x / 3x)
            digitalWrite(PIN_LED_HIJAU, customBeepIsOn ? HIGH : LOW);
            digitalWrite(PIN_LED_MERAH, customBeepIsOn ? LOW : HIGH);
        } else {
            // Normal Disarm: Merah nyala solid, kedip sejenak jika ada bip peringatan
            digitalWrite(PIN_LED_HIJAU, LOW);
            digitalWrite(PIN_LED_MERAH, customBeepIsOn ? LOW : HIGH);
        }
    }
}

// ============================================================================
// 4. FUNGSI FHSS, EEPROM, DAN KONFIGURASI REGISTER NRF24L01 (GAYA RF24)
// ============================================================================
// Meracik 4 saluran lompatan FHSS dari 1 angka bibit (100% identik dengan RX)
static void customGenerateFHSS(uint8_t seed) {
    for (uint8_t i = 0; i < CUSTOM_FHSS_LEN; i++) {
        customFhssChannels[i] = 20 + ((seed - 20 + (i * 23)) % 80);
    }
    customHopIndex = 0;
}

// Memuat atau membuat baru alamat pipa, channel, mode, dan tipe motor dari EEPROM (Byte 20-28)
static void customLoadEEPROM() {
    if (EEPROM.read(EE_CUSTOM_MAGIC) != 0xAC) {
        Serial.println(F("[CUSTOM-EEPROM] Slot baru! Membuat alamat pipa & channel acak..."));
        customSeedChannel = random(20, 95);
        for (uint8_t i = 0; i < 5; i++) {
            customPipe[i] = 'A' + random(0, 26); // 5 huruf kapital acak A-Z
        }
        customModeIndex       = 1; // Default Mode 1 (Pesawat Normal / Elevon)
        customActiveMotorType = 1; // Default MOSFET Brushed (aman 0V saat awal)

        EEPROM.update(EE_CUSTOM_MAGIC, 0xAC);
        EEPROM.update(EE_CUSTOM_CH, customSeedChannel);
        for (uint8_t i = 0; i < 5; i++) {
            EEPROM.update(EE_CUSTOM_PIPE + i, customPipe[i]);
        }
        EEPROM.update(EE_CUSTOM_MODE, customModeIndex);
        EEPROM.update(EE_CUSTOM_MOTOR, customActiveMotorType);
    } else {
        customSeedChannel = EEPROM.read(EE_CUSTOM_CH);
        if (customSeedChannel < 20 || customSeedChannel > 94) customSeedChannel = 76;
        for (uint8_t i = 0; i < 5; i++) {
            customPipe[i] = EEPROM.read(EE_CUSTOM_PIPE + i);
        }
        customModeIndex = EEPROM.read(EE_CUSTOM_MODE);
        if (customModeIndex < 1 || customModeIndex > 3) {
            customModeIndex = 1;
            EEPROM.update(EE_CUSTOM_MODE, customModeIndex);
        }
        customActiveMotorType = EEPROM.read(EE_CUSTOM_MOTOR);
        if (customActiveMotorType > 1) {
            customActiveMotorType = 1; // Default aman ke MOSFET jika byte masih 0xFF
            EEPROM.update(EE_CUSTOM_MOTOR, customActiveMotorType);
        }
    }

    customGenerateFHSS(customSeedChannel);

    Serial.print(F("[CUSTOM-EEPROM] Pipe: "));
    for (uint8_t i = 0; i < 5; i++) Serial.print((char)customPipe[i]);
    Serial.print(F(" | Seed Ch: ")); Serial.print(customSeedChannel);
    Serial.print(F(" | FHSS 4-Ch: "));
    for (uint8_t i = 0; i < CUSTOM_FHSS_LEN; i++) {
        Serial.print(customFhssChannels[i]); Serial.print(' ');
    }
    Serial.print(F("| Mode: ")); Serial.print(customModeIndex);
    Serial.print(F("(RX:")); Serial.print(customGetRxAuxFlag(customModeIndex));
    Serial.print(F(") | Motor: "));
    Serial.println(customActiveMotorType == 1 ? F("MOSFET BRUSHED") : F("ESC BRUSHLESS"));
}

// Menyetel seluruh register NRF24L01 agar 100% kompatibel dengan library RF24 di RX
static void customConfigureRadio(const uint8_t* addr5, uint8_t channel) {
    CE_off; // Masuk Standby-I sebelum mengubah konfigurasi register

    // CONFIG (0x00) = 0x0E -> EN_CRC (bit 3) | CRCO 16-bit (bit 2) | PWR_UP (bit 1) | PRIM_RX=0 (TX)
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0E);
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);      // Auto-ACK aktif di semua pipe
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x03);  // Pipe 0 & Pipe 1 aktif
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // Lebar alamat 5 byte
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x13); // Retry: ARD=500us (1), ARC=3x (3) -> sama dgn setRetries(1,3)
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);   // Setel saluran frekuensi
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // Kecepatan transfer 1 Mbps
    NRF24L01_SetPower(RF_POWER);                     // Daya pancar penuh (PA_HIGH / 80mW)
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Bersihkan flag RX_DR, TX_DS, MAX_RT

    // Buka kunci register FEATURE (diperlukan untuk chip nRF24L01+ dan klon Si24R1)
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);      // Dynamic Payload aktif di semua pipe
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x06);    // EN_DPL (bit 2) | EN_ACK_PAY (bit 1) = 0x06

    // Wajib setel RX_ADDR_P0 == TX_ADDR agar paket balasan Auto-ACK & Telemetri diterima!
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, addr5, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, addr5, 5);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    delay(5); // Tunggu osilator PLL stabil
}

// Mengirim 1 paket secara blocking singkat (~1-3 ms) dan menyedot ACK Payload jika ada
static bool customRadioWrite(uint8_t* payload, uint8_t len) {
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();

    NRF24L01_WritePayload(payload, len);
    CE_on; // Mulai pancaran

    unsigned long startUs = micros();
    uint8_t status = 0;
    while (micros() - startUs < 4000UL) {
        status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
        if (status & (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT))) {
            break;
        }
    }
    CE_off; // Kembali ke Standby-I agar aman saat pindah channel FHSS

    bool ackSukses = (status & _BV(NRF24L01_07_TX_DS)) != 0;

    if (ackSukses) {
        // Cek apakah RX menitipkan paket Telemetri (flag RX_DR bit 6 = 0x40)
        if (status & _BV(NRF24L01_07_RX_DR)) {
            NRF24L01_ReadPayload((uint8_t*)&customRxTelem, sizeof(CustomTelemetryData));
            NRF24L01_FlushRx();
        }
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        return true;
    } else {
        NRF24L01_FlushTx();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        return false;
    }
}

// ============================================================================
// 5. FUNGSI PEMBACA SAKLAR MICROZONE (CH.5 3-POS & CH.6 2-POS) & STIK
// ============================================================================
// Pembaca CH.5 (3-Posisi): 0 = BAWAH (< 1300), 1 = TENGAH (1300..1700), 2 = ATAS (> 1700)
static uint8_t bacaSaklar3Pos(uint16_t nilaiPpm, bool dibalik) {
    uint8_t pos = 1;
    if (nilaiPpm < PPM_MIN_COMMAND)      pos = 0;
    else if (nilaiPpm > PPM_MAX_COMMAND) pos = 2;
    if (dibalik) pos = 2 - pos;
    return pos;
}

// Pembaca CH.6 (2-Posisi dengan histeresis): cek apakah posisi ON (ATAS) atau OFF (BAWAH)
static bool isCh6On(uint16_t nilaiPpm) {
    return CH6_REVERSE ? (nilaiPpm < PPM_MIN_COMMAND) : (nilaiPpm > PPM_MAX_COMMAND);
}

static bool isCh6Off(uint16_t nilaiPpm) {
    return CH6_REVERSE ? (nilaiPpm > PPM_MAX_COMMAND) : (nilaiPpm < PPM_MIN_COMMAND);
}

// Konversi PPM stik (1000..2000) menjadi -127..+127 dengan deadzone tengah 30 us
// Semua channel (Rudder, Elevator, Aileron) diteruskan murni agar saklar MIX+ELEVON MC6C bekerja penuh
static int8_t mapPpmAxis(uint16_t val) {
    val = constrain(val, PPM_MIN, PPM_MAX);
    if (val >= 1485 && val <= 1515) return 0; // Deadzone tengah anti-getar
    if (val < 1485) return (int8_t)map(val, PPM_MIN, 1485, -127, 0);
    return (int8_t)map(val, 1515, PPM_MAX, 0, 127);
}

// ============================================================================
// 6. PROSES BINDING 10 DETIK (CHANNEL 10, ALAMAT "BIND0")
// ============================================================================
static void customExecuteBinding() {
    Serial.println(F("[CUSTOM-BIND] Saklar CH5+CH6 ATAS terdeteksi! Mulai Binding 10 Detik..."));
    customConfigureRadio(CUSTOM_BIND_ADDR, CUSTOM_BIND_CH);
    cekRegisterNRF24("Mode Binding CustomRX (Ch 10)");

    CustomBindPayload bp;
    memcpy(bp.pipeAddr, customPipe, 5);
    bp.opChannel = customSeedChannel;
    bp.checkSum  = customSeedChannel ^ bp.pipeAddr[0] ^ bp.pipeAddr[4];

    unsigned long bindStart = millis();
    while (millis() - bindStart < 10000UL) {
        customRadioWrite((uint8_t*)&bp, sizeof(CustomBindPayload));

        // Efek disko silang Merah-Hijau + Buzzer bernyanyi (identik dengan V930)
        nyalakanBuzzer(true);
        digitalWrite(PIN_LED_MERAH, HIGH);
        digitalWrite(PIN_LED_HIJAU, LOW);
        delay(40);

        nyalakanBuzzer(false);
        digitalWrite(PIN_LED_MERAH, LOW);
        digitalWrite(PIN_LED_HIJAU, HIGH);
        delay(80);
    }

    digitalWrite(PIN_LED_HIJAU, LOW);
    digitalWrite(PIN_LED_MERAH, HIGH);
    beepBlocking(500);
    Serial.println(F("[CUSTOM-BIND] Selesai 10 detik! Kembali ke FHSS Operasional."));
    Serial.println(F("[SAFETY] Turunkan saklar CH.5 ke BAWAH untuk membuka kunci Arming."));
}

// ============================================================================
// 7. INISIALISASI PROTOKOL CUSTOM RX (DIPANGGIL DARI init_protocol())
// ============================================================================
void CustomRX_init() {
    // 1. Muat alamat pipa, 4 saluran FHSS, mode, dan tipe motor terakhir dari EEPROM
    customLoadEEPROM();

    // 2. Pastikan status awal selalu DISARMED dan terkunci
    customIsArmed                = false;
    customMustDisarmFirst        = true;
    customPendingMotor           = 255;
    customRxTelem.batteryStatus  = 0;
    customRxTelem.failsafeActive = 1;

    // 3. Perbarui bacaan saklar CH.5 dan CH.6 saat booting
    update_ppm();
    uint8_t posCH5 = bacaSaklar3Pos(ppm[AUX1], CH5_REVERSE);
    bool    ch6On  = isCh6On(ppm[AUX2]);

    // Kunci CH.6 jika saat booting sudah di posisi ATAS (mencegah mode loncat sendiri)
    customCh6WasHigh = ch6On;

    // 4. Cek Pemicu Binding: HANYA aktif jika CH.5 == ATAS (2) DAN CH.6 == ON/ATAS
    if (posCH5 == 2 && ch6On) {
        customExecuteBinding();
        customCh6WasHigh = true; // Wajib turunkan CH.6 dulu sebelum bisa ganti mode
    } else {
        if (posCH5 == 0) {
            customMustDisarmFirst = false; // CH.5 sudah di bawah -> kunci arming terbuka
        } else {
            Serial.println(F("[SAFETY] Saklar CH.5 belum di BAWAH! Turunkan CH.5 dulu sebelum Arm."));
            beepBlocking(100); delay(80); beepBlocking(100);
        }
    }

    // 5. Konfigurasi radio ke alamat operasional & channel FHSS pertama (indeks 0)
    customHopIndex = 0;
    customConfigureRadio(customPipe, customFhssChannels[customHopIndex]);
    cekRegisterNRF24("Operasional FHSS CustomRX");

    // 6. Penanda Awal Mode Aktif (1x, 2x, atau 3x Bip + Kedip LED Hijau)
    delay(250);
    for (uint8_t i = 0; i < customModeIndex; i++) {
        digitalWrite(PIN_LED_MERAH, LOW);
        digitalWrite(PIN_LED_HIJAU, HIGH);
        beepBlocking(110);
        digitalWrite(PIN_LED_HIJAU, LOW);
        digitalWrite(PIN_LED_MERAH, HIGH);
        delay(130);
    }

    customLastAckTime = millis();
    Serial.print(F("[CUSTOM-TX] Siap! Kondisi: DISARMED | Mode: "));
    Serial.print(customModeIndex);
    Serial.print(F("(RX:")); Serial.print(customGetRxAuxFlag(customModeIndex));
    Serial.print(F(") | Motor: "));
    Serial.println(customActiveMotorType == 1 ? F("MOSFET BRUSHED") : F("ESC BRUSHLESS"));
}

// ============================================================================
// 8. LOOP UTAMA PROTOKOL CUSTOM RX (DIPANGGIL TIAP 30ms OLEH loop())
// ============================================================================
uint32_t process_CustomRX() {
    uint32_t nextTimeout = micros() + CUSTOM_TX_INTERVAL; // Jadwal kirim 30ms berikutnya
    unsigned long now    = millis();

    // 1. Jalankan pemroses Buzzer & LED Merah/Hijau di latar belakang
    customHandleAudioVisual();

    // 2. Baca posisi saklar bahu Microzone (CH.5 & CH.6) dan nilai stik PPM
    uint8_t posCH5 = bacaSaklar3Pos(ppm[AUX1], CH5_REVERSE);
    bool    ch6On  = isCh6On(ppm[AUX2]);
    bool    ch6Off = isCh6Off(ppm[AUX2]);

    // Hitung gas murni (0..255) dengan deadband bawah <= 1030 us = 0
    uint8_t thrVal = 0;
    if (ppm[THROTTLE] > 1030) {
        thrVal = (uint8_t)constrain(map(ppm[THROTTLE], 1030, PPM_MAX, 0, 255), 0, 255);
    }

    // 3. LOGIKA SAKLAR CH.5:
    //    BAWAH (0)  = DISARM instan (tipe motor terakhir dipertahankan!)
    //    TENGAH (1) = ARM -> MOSFET Brushed (motorType = 1)
    //    ATAS (2)   = ARM -> ESC Brushless  (motorType = 0)
    //    Dilengkapi jeda konfirmasi 250ms agar geser cepat Atas <-> Bawah tidak
    //    keliru mengubah tipe motor saat melintasi posisi Tengah.
    if (posCH5 == 0) {
        // Batalkan antrean pergantian motor (jika tadi jari hanya melintasi Tengah)
        customPendingMotor = 255;

        if (customIsArmed) {
            customIsArmed = false;
            customBeepN(1, 200, 80, false);
            Serial.print(F("[MOTOR] DISARMED (Tetap di "));
            Serial.print(customActiveMotorType == 1 ? F("MOSFET BRUSHED") : F("ESC BRUSHLESS"));
            Serial.println(F(")"));
        }
        customMustDisarmFirst = false;
    } else {
        // Posisi TENGAH (1) -> MOSFET Brushed (1) | Posisi ATAS (2) -> ESC Brushless (0)
        uint8_t targetMotorType = (posCH5 == 1) ? 1 : 0;

        if (!customIsArmed && !customMustDisarmFirst) {
            // Syarat mutlak sebelum boleh ARMED: Stik gas WAJIB di nol!
            if (thrVal > 10) {
                customBeepN(2, 90, 80, false); // 2x bip peringatan
                customMustDisarmFirst = true;  // Wajib turunkan CH.5 dulu sebelum coba lagi
                customPendingMotor    = 255;
                Serial.println(F("[WARNING] Arming DITOLAK! Turunkan stik gas ke nol & reset CH.5!"));
            } else {
                // Tunggu saklar mantap 250ms di Tengah atau Atas sebelum mengunci ARM
                if (customPendingMotor != targetMotorType) {
                    customPendingMotor = targetMotorType;
                    customMotorTimer   = now;
                } else if (now - customMotorTimer >= CH5_MOTOR_SETTLE_MS) {
                    customActiveMotorType = targetMotorType;
                    // EEPROM.update hanya menulis jika nilainya BERBEDA (0 siklus tulis jika sama)
                    EEPROM.update(EE_CUSTOM_MOTOR, customActiveMotorType);
                    customIsArmed      = true;
                    customPendingMotor = 255;
                    customBeepN(1, 400, 80, false);
                    Serial.print(F("[MOTOR] ARMED! Tipe Motor: "));
                    Serial.println(customActiveMotorType == 1 ? F("MOSFET BRUSHED") : F("ESC BRUSHLESS"));
                }
            }
        }
        else if (customIsArmed && thrVal <= 10) {
            // SUDAH ARMED & GAS MASIH NOL (DI DARAT):
            // Izinkan pindah Tengah (MOSFET) <-> Atas (ESC) setelah diam mantap 250ms
            if (targetMotorType != customActiveMotorType) {
                if (customPendingMotor != targetMotorType) {
                    customPendingMotor = targetMotorType;
                    customMotorTimer   = now;
                } else if (now - customMotorTimer >= CH5_MOTOR_SETTLE_MS) {
                    customActiveMotorType = targetMotorType;
                    EEPROM.update(EE_CUSTOM_MOTOR, customActiveMotorType);
                    customPendingMotor = 255;
                    // Penanda bunyi: 2x bip pendek = MOSFET Brushed, 1x bip pendek = ESC Brushless
                    if (customActiveMotorType == 1) customBeepN(2, 90, 80, false);
                    else                            customBeepN(1, 130, 80, false);
                    Serial.print(F("[MOTOR] Pindah Tipe Motor (Gas Nol): "));
                    Serial.println(customActiveMotorType == 1 ? F("MOSFET BRUSHED") : F("ESC BRUSHLESS"));
                }
            } else {
                customPendingMotor = 255;
            }
        } else {
            // Saat gas sudah naik (> 10 / sedang terbang), kunci tipe motor demi keselamatan
            customPendingMotor = 255;
        }
    }

    // 4. LOGIKA SAKLAR CH.6 (2-POSISI TAKTIL): PINDAH MODE BERURUTAN 1 -> 2 -> 3 -> 1
    // Hanya aktif saat DISARMED agar aman; wajib kembali ke BAWAH (OFF) untuk pindah lagi
    if (!customIsArmed) {
        if (ch6On && !customCh6WasHigh) {
            customCh6WasHigh = true; // Kunci saklar sampai dikembalikan ke BAWAH (OFF)

            // Geser urutan mode: 1 -> 2 -> 3 -> kembali ke 1
            customModeIndex = (customModeIndex % 3) + 1;
            EEPROM.update(EE_CUSTOM_MODE, customModeIndex); // Hanya tulis jika berubah

            // Bunyikan 1x, 2x, atau 3x bip + kedip LED Hijau (non-blocking)
            customBeepN(customModeIndex, 100, 100, true);

            Serial.print(F("[MODE] Pindah ke Mode "));
            Serial.print(customModeIndex);
            if (customModeIndex == 1)      Serial.println(F(" (Pesawat Normal/Elevon -> auxFlag: 1)"));
            else if (customModeIndex == 2) Serial.println(F(" (Diferensial 2 Motor  -> auxFlag: 2)"));
            else                           Serial.println(F(" (Mobil RC Propo       -> auxFlag: 4)"));
        } else if (ch6Off) {
            customCh6WasHigh = false; // Saklar kembali ke BAWAH -> buka kunci untuk klik berikutnya
        }
    } else {
        // Saat ARMED, tetap pantau posisi CH.6 agar saat nanti DISARM tidak langsung loncat
        if (ch6On)       customCh6WasHigh = true;
        else if (ch6Off) customCh6WasHigh = false;
    }

    // 5. SUSUN PAKET 7 BYTE ControlData (AETR Murni + Terjemahan auxFlag RX)
    customTxData.throttle  = customIsArmed ? thrVal : 0; // Paksa gas 0 saat Disarmed
    customTxData.rudder    = mapPpmAxis(ppm[RUDDER]);    // Murni dari PPM (Mode 1 kompatibel MIX)
    customTxData.elevator  = mapPpmAxis(ppm[ELEVATOR]);  // Murni dari PPM (Mode 1 kompatibel MIX)
    customTxData.aileron   = mapPpmAxis(ppm[AILERON]);   // Murni dari PPM (Mode 1 kompatibel MIX)
    customTxData.isArmed   = customIsArmed ? 1 : 0;
    customTxData.auxFlag   = customGetRxAuxFlag(customModeIndex); // 1->1, 2->2, 3->4
    customTxData.motorType = customActiveMotorType;

    // 6. TEMBAKKAN PAKET KE UDARA & TANGKAP TELEMETRI BALASAN
    if (customRadioWrite((uint8_t*)&customTxData, sizeof(CustomControlData))) {
        customLastAckTime = now; // Kontak dengan RX berhasil di channel ini
    }

    // 7. LOMPAT KE CHANNEL FHSS BERIKUTNYA (0 -> 1 -> 2 -> 3 -> 0)
    customHopIndex = (customHopIndex + 1) % CUSTOM_FHSS_LEN;
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, customFhssChannels[customHopIndex]);

    // 8. EVALUASI STATUS KONEKSI & ALARM BATERAI 1S RX LEMAH
    bool isRxConnected = (now - customLastAckTime < 1000UL);
    if (!isRxConnected) {
        customRxTelem.batteryStatus = 0; // Bungkam alarm palsu saat RX belum nyala / putus
    } else if (customIsArmed && customRxTelem.batteryStatus == 1) {
        if (now - customLastAlarmTime >= 2500UL) {
            customBeepN(1, 500, 100, false); // Alarm panjang tiap 2.5 detik saat baterai RX kritis
            customLastAlarmTime = now;
            Serial.println(F("[ALARM] BATERAI 1S RX LEMAH (<= 3.5V)! SEGERA MENDARAT!"));
        }
    }

    // 9. LOG SERIAL DEBUGGING BERKALA (Tiap 500ms - otomatis terhapus saat AKTIFKAN_SERIAL == 0)
    if (now - customLastDebugPrint >= 500UL) {
        customLastDebugPrint = now;
        Serial.print(F("[TX] Link:"));  Serial.print(isRxConnected ? F("OK") : F("NO"));
        Serial.print(F(" | Arm:"));    Serial.print(customTxData.isArmed);
        Serial.print(F(" | M:"));      Serial.print(customModeIndex);
        Serial.print(F("(RX:"));       Serial.print(customTxData.auxFlag);
        Serial.print(F(") | Mot:"));   Serial.print(customTxData.motorType == 1 ? F("MOS") : F("ESC"));
        Serial.print(F(" | T:"));      Serial.print(customTxData.throttle);
        Serial.print(F(" R:"));        Serial.print(customTxData.rudder);
        Serial.print(F(" E:"));        Serial.print(customTxData.elevator);
        Serial.print(F(" A:"));        Serial.print(customTxData.aileron);
        Serial.print(F(" | BatRX:"));  Serial.println(customRxTelem.batteryStatus);
    }

    return nextTimeout;
}