#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SdFat.h>

//===============================================
// Teensy 4.1 + SdFat
// 1msタイマー割り込み + バッファリング + CSV書き込み
//===============================================

// 前方宣言
template<typename T>
class SDLogger;

// LogEntry構造体のテンプレート - ユーザーデータ型 + タイムスタンプを含む
template<typename T>
struct LogEntry {
    uint32_t timestamp_ms;  // タイムスタンプ(ミリ秒単位)
    T data;                 // ユーザー定義のデータ構造体
};

// SDLoggerクラスのテンプレート実装
template<typename T>
class SDLogger {
public:
    using LogData_t = LogEntry<T>;

    // コンストラクタ
    SDLogger(const char* filename, uint16_t bufferLength = 256, uint16_t flushInterval = 100);
    
    // デストラクタ
    ~SDLogger();
    
    // 初期化
    bool begin();

    bool begin(const char* filename);
    
    // タイマー割り込みの開始（指定間隔 [μs]）
    bool startLogging(uint32_t interval_us = 1000);
    
    // タイマー割り込みの停止
    void stopLogging();
    
    // SDカードへの書き込み処理（loop内で呼び出し）
    void update();
    
    // ファイルを閉じる
    void close();

    // bufferをクリア
    void clearBuffer();
    
    // 現在のバッファ内データ数を取得
    uint16_t getBufferCount() const;
    
    // バッファが満杯かどうか
    bool isBufferFull() const;
    
    // 現在の記録用データへの参照を取得（ユーザーがデータを更新するため）
    T& getCurrentData();
    
    // CSVのヘッダー設定
    void setCsvHeader(const char* header);
    
    // ユーザー定義のCSV変換関数の型定義
    using CsvConverter = void (*)(const T& data, Print& output);
    
    // CSV変換関数の設定
    void setCsvConverter(CsvConverter converter);
    
private:
    // 設定パラメータ
    const char* _filename;    // CSVファイル名
    uint16_t _bufferLength;   // リングバッファの長さ
    uint16_t _flushInterval;  // どのくらいの頻度でflushするか
    
    // SdFat と CSVファイル
    SdFat _sd;
    FsFile _csvFile;
    
    // IntervalTimer（Teensy固有）
    IntervalTimer _timer;
    
    // リングバッファ本体
    LogData_t* _ringBuffer;
    
    // リングバッファのポインタ(先頭と末尾)
    volatile uint16_t _ringHead;  // 書き込み側
    volatile uint16_t _ringTail;  // 読み出し側
    
    // 現在のデータ
    T _currentData;
    
    // flushのタイミング管理用カウンタ
    uint32_t _flushCounter;
    
    // CSVヘッダー
    char* _csvHeader;
    
    // CSV変換関数
    CsvConverter _csvConverter;
    
    // デフォルトのCSV変換関数（T型に対応したwrite関数が必要）
    static void defaultCsvConverter(const T& data, Print& output);
    
    // 割り込みハンドラ
    static void isrTrampoline();
    
    // 実際の割り込み処理
    void handleInterrupt();
    
    // ISRからアクセスするための静的インスタンスポインタ配列
    static SDLogger<T>* _instances[5]; // 最大5つのインスタンスをサポート（必要に応じて変更）
    static uint8_t _instanceCount;
};

// 静的メンバの初期化
template<typename T>
SDLogger<T>* SDLogger<T>::_instances[5] = {nullptr};

template<typename T>
uint8_t SDLogger<T>::_instanceCount = 0;

//===============================================
// SDLoggerクラスの実装
//===============================================

// コンストラクタ
template<typename T>
SDLogger<T>::SDLogger(const char* filename, uint16_t bufferLength, uint16_t flushInterval) 
    : _filename(filename), 
      _bufferLength(bufferLength), 
      _flushInterval(flushInterval),
      _ringHead(0),
      _ringTail(0),
      _flushCounter(0),
      _csvHeader(nullptr),
      _csvConverter(defaultCsvConverter)
{
    // リングバッファのメモリ確保
    _ringBuffer = new LogData_t[_bufferLength];
    
    // インスタンスの登録
    if (_instanceCount < 5) {
        _instances[_instanceCount++] = this;
    }
}

// デストラクタ
template<typename T>
SDLogger<T>::~SDLogger() {
    // ファイルが開いていれば閉じる
    close();
    
    // タイマー停止
    stopLogging();
    
    // リングバッファの解放
    delete[] _ringBuffer;
    
    // CSVヘッダーの解放
    if (_csvHeader) {
        delete[] _csvHeader;
    }
    
    // インスタンスの登録解除
    for (uint8_t i = 0; i < _instanceCount; i++) {
        if (_instances[i] == this) {
            // 最後のインスタンスを現在の位置に移動
            _instances[i] = _instances[--_instanceCount];
            _instances[_instanceCount] = nullptr;
            break;
        }
    }
}

// 初期化
template<typename T>
bool SDLogger<T>::begin() {
    // SdFat (SDIO)初期化
    if (!_sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("SD initialization failed");
        return false;
    }
    
    // 既存のファイルがあれば削除(オプション)
    _sd.remove(_filename);
    
    // CSVファイルを新規作成
    _csvFile = _sd.open(_filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (!_csvFile) {
        Serial.println("Failed to create CSV file");
        return false;
    }
    
    // CSVヘッダーがあれば書き込む
    if (_csvHeader) {
        _csvFile.println(_csvHeader);
    }
    
    return true;
}

template<typename T>
bool SDLogger<T>::begin(const char* filename) 
{
    _filename = filename;
    return begin();
}

// タイマー割り込みの開始
template<typename T>
bool SDLogger<T>::startLogging(uint32_t interval_us) {
    // タイマー開始
    _timer.begin(isrTrampoline, interval_us);
    return true;
}

// タイマー割り込みの停止
template<typename T>
void SDLogger<T>::stopLogging() {
    _timer.end();
}

// 割り込みハンドラ（static関数）- 登録された全インスタンスの割り込み処理を呼び出す
template<typename T>
void SDLogger<T>::isrTrampoline() {
    // 全ての登録インスタンスの割り込み処理を呼び出す
    for (uint8_t i = 0; i < _instanceCount; i++) {
        if (_instances[i]) {
            _instances[i]->handleInterrupt();
        }
    }
}

// 実際の割り込み処理
template<typename T>
void SDLogger<T>::handleInterrupt() {
    // タイムスタンプを取得
    uint32_t t_ms = millis();
    
    // 次に書き込む先を計算
    uint16_t nextHead = (_ringHead + 1) % _bufferLength;
    
    // バッファが満杯の場合は最古のデータを捨てる
    if (nextHead == _ringTail) {
        _ringTail = (_ringTail + 1) % _bufferLength;
    }
    
    // データ書き込み
    _ringBuffer[_ringHead].timestamp_ms = t_ms;
    _ringBuffer[_ringHead].data = _currentData;
    
    // ヘッドを進める
    _ringHead = nextHead;
}

// SDカードへの書き込み処理
template<typename T>
void SDLogger<T>::update() {
    // リングバッファにデータがあれば1件ずつCSVへ書き込み
    while (_ringTail != _ringHead) {
        // 取り出し
        uint32_t t_ms = _ringBuffer[_ringTail].timestamp_ms;
        const T& data = _ringBuffer[_ringTail].data;
        
        // CSV出力（行の先頭にタイムスタンプ）
        _csvFile.print(t_ms);
        _csvFile.print(',');
        
        // ユーザー定義の変換関数を使用してデータをCSVに出力
        if (_csvConverter) {
            _csvConverter(data, _csvFile);
        }
        
        _csvFile.println();
        
        // Tailを進める
        _ringTail = (_ringTail + 1) % _bufferLength;
        
        // ある程度たまったらflush
        _flushCounter++;
        if (_flushCounter >= _flushInterval) {
            _csvFile.flush();
            _flushCounter = 0;
        }
    }
}

// ファイルを閉じる
template<typename T>
void SDLogger<T>::close() {
    if (_csvFile) {
        _csvFile.flush();
        _csvFile.close();
    }
    clearBuffer();
}

template<typename T>
void SDLogger<T>::clearBuffer()
{
    _ringHead = 0;
    _ringTail = 0;
}

// 現在のバッファ内データ数を取得
template<typename T>
uint16_t SDLogger<T>::getBufferCount() const {
    if (_ringHead >= _ringTail) {
        return _ringHead - _ringTail;
    } else {
        return _bufferLength - _ringTail + _ringHead;
    }
}

// バッファが満杯かどうか
template<typename T>
bool SDLogger<T>::isBufferFull() const {
    uint16_t nextHead = (_ringHead + 1) % _bufferLength;
    return nextHead == _ringTail;
}

// 現在の記録用データへの参照を取得
template<typename T>
T& SDLogger<T>::getCurrentData() {
    return _currentData;
}

// CSVのヘッダー設定
template<typename T>
void SDLogger<T>::setCsvHeader(const char* header) {
    // 古いヘッダーがあれば解放
    if (_csvHeader) {
        delete[] _csvHeader;
    }
    
    // 新しいヘッダーをコピー
    size_t headerLen = strlen(header);
    _csvHeader = new char[headerLen + 1];
    strcpy(_csvHeader, header);
}

// CSV変換関数の設定
template<typename T>
void SDLogger<T>::setCsvConverter(CsvConverter converter) {
  _csvConverter = converter ? converter : defaultCsvConverter;
}

// デフォルトのCSV変換関数
// 注意: これはコンパイルエラーになる可能性があります
// ユーザーは自分のデータ型に合わせた変換関数を提供する必要があります
template<typename T>
void SDLogger<T>::defaultCsvConverter(const T& data, Print& output) {
    // データ型Tに対してどのように出力するかはユーザーが定義する必要がある
    // 例としてT::writeToCSV()メソッドを使用することを想定
    data.writeToCSV(output);
}

#endif // SD_LOGGER_H