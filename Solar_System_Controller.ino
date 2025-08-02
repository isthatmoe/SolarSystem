/*
==================================================================
          دارة التحكم الاوتوماتيكي بأحمال انظمة الطاقة الشمسية
                      الاصدار: 1.0.0
                  المطور: نظام التحكم الذكي
==================================================================

الميزات:
- تحكم في مخرجين ريلاي (GPIO 14, GPIO 5)
- قراءة جهد البطارية مع معايرة متقدمة
- مراقبة الشبكة العامة (GPIO 4)
- تايمرات يومية متعددة لكل مخرج
- واجهة ويب عربية كاملة
- حفظ الاعدادات في EEPROM

==================================================================
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <time.h>

// ================= تعريف المداخل والمخارج =================
#define RELAY_1_PIN 14        // الريلاي الأول
#define RELAY_2_PIN 5         // الريلاي الثاني
#define GRID_SENSOR_PIN 4     // حساس الشبكة العامة
#define ADC_PIN A0            // مدخل قراءة الجهد

// ================= اعدادات النظام العامة =================
#define NUM_SAMPLES 60        // عدد عينات قراءة ADC
#define EEPROM_SIZE 512       // حجم EEPROM المخصص
#define MAX_CALIBRATION_POINTS 20  // عدد نقاط المعايرة القصوى
#define GRID_STABILITY_TIME 60000  // وقت استقرار الشبكة (دقيقة)
#define RELAY_DELAY_TIME 3000      // تأخير بين تشغيل الريلايات

// ================= هيكل بيانات نقطة المعايرة =================
struct CalibrationPoint {
  float adcValue;
  float realVoltage;
};

// ================= هيكل بيانات التايمر =================
struct DailyTimer {
  bool enabled;
  int startHour;
  int startMinute;
  int endHour;
  int endMinute;
};

// ================= هيكل بيانات اعدادات الريلاي =================
struct RelaySettings {
  float onVoltage;          // جهد التشغيل
  float offVoltage;         // جهد الايقاف
  bool manualOverride;      // التحكم اليدوي
  bool manualState;         // حالة التحكم اليدوي
  DailyTimer timers[3];     // ثلاث تايمرات يومية
};

// ================= المتغيرات العامة =================
ESP8266WebServer server(80);

// اعدادات الشبكة
String apSSID = "Solar System";
String apPassword = "";
bool apPasswordEnabled = false;

// جدول المعايرة
CalibrationPoint calibrationTable[MAX_CALIBRATION_POINTS] = {
  {0.00, 0.0},
  {78.0, 5.016},
  {102.0, 6.53},
  {133.00, 8.40},
  {147.0, 9.12},
  {161.0, 10.04},
  {198.0, 12.43},
  {240.0, 15.07},
  {317.0, 19.98},
  {368.0, 23.27},
  {399.0, 25.16},
  {522.0, 32.77}
};
int calibrationPointsCount = 12;

// اعدادات الريلايات
RelaySettings relaySettings[2];

// حالة النظام
float currentVoltage = 0.0;
bool gridAvailable = false;
bool gridStable = false;
unsigned long gridStableStartTime = 0;
bool relay1State = false;
bool relay2State = false;
unsigned long lastRelayChange = 0;

// اعدادات النظام
bool systemEnabled = true;
bool antiChattering = true;
int relayDelaySeconds = 3;

// متغيرات التوقيت
unsigned long previousMillis = 0;
const long interval = 1000; // فترة التحديث بالميلي ثانية

// ================= دوال المعايرة =================
float takeAverageADCReading() {
  float total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    total += analogRead(ADC_PIN);
    delay(2);
  }
  return total / NUM_SAMPLES;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float getCorrectedVoltage(float currentADC) {
  // التحقق من وجود نقاط معايرة
  if (calibrationPointsCount < 2) return -1.0;
  
  // التعامل مع القيم خارج النطاق
  if (currentADC <= calibrationTable[0].adcValue) {
    return mapFloat(currentADC, calibrationTable[0].adcValue, calibrationTable[1].adcValue,
                   calibrationTable[0].realVoltage, calibrationTable[1].realVoltage);
  }
  
  if (currentADC >= calibrationTable[calibrationPointsCount-1].adcValue) {
    return mapFloat(currentADC, calibrationTable[calibrationPointsCount-2].adcValue, 
                   calibrationTable[calibrationPointsCount-1].adcValue,
                   calibrationTable[calibrationPointsCount-2].realVoltage, 
                   calibrationTable[calibrationPointsCount-1].realVoltage);
  }
  
  // البحث عن المقطع المناسب
  for (int i = 0; i < calibrationPointsCount - 1; i++) {
    float adc1 = calibrationTable[i].adcValue;
    float volt1 = calibrationTable[i].realVoltage;
    float adc2 = calibrationTable[i+1].adcValue;
    float volt2 = calibrationTable[i+1].realVoltage;
    
    if (adc1 <= currentADC && currentADC <= adc2) {
      return mapFloat(currentADC, adc1, adc2, volt1, volt2);
    }
  }
  
  return -1.0;
}

// ================= دوال التايمر =================
bool isTimeInRange(DailyTimer timer) {
  if (!timer.enabled) return false;
  
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  
  int currentMinutes = timeinfo->tm_hour * 60 + timeinfo->tm_min;
  int startMinutes = timer.startHour * 60 + timer.startMinute;
  int endMinutes = timer.endHour * 60 + timer.endMinute;
  
  if (startMinutes <= endMinutes) {
    return (currentMinutes >= startMinutes && currentMinutes <= endMinutes);
  } else {
    // التايمر يتجاوز منتصف الليل
    return (currentMinutes >= startMinutes || currentMinutes <= endMinutes);
  }
}

bool shouldRelayBeOn(int relayIndex) {
  // التحقق من التحكم اليدوي
  if (relaySettings[relayIndex].manualOverride) {
    return relaySettings[relayIndex].manualState;
  }
  
  // التحقق من التايمرات
  for (int i = 0; i < 3; i++) {
    if (isTimeInRange(relaySettings[relayIndex].timers[i])) {
      return true;
    }
  }
  
  // التحقق من جهد البطارية
  if (currentVoltage >= relaySettings[relayIndex].onVoltage) {
    return true;
  }
  
  if (currentVoltage <= relaySettings[relayIndex].offVoltage) {
    return false;
  }
  
  // المحافظة على الحالة الحالية اذا كان الجهد بين القيمتين
  return (relayIndex == 0) ? relay1State : relay2State;
}

// ================= دوال التحكم في الريلايات =================
void updateRelayStates() {
  if (!systemEnabled) return;
  if (!gridStable && !gridAvailable) return;
  
  unsigned long currentTime = millis();
  
  // تطبيق تأخير بين تشغيل الريلايات
  if (currentTime - lastRelayChange < (relayDelaySeconds * 1000)) {
    return;
  }
  
  bool newRelay1State = shouldRelayBeOn(0);
  bool newRelay2State = shouldRelayBeOn(1);
  
  // تطبيق منطق منع التكرار
  if (antiChattering) {
    // تنفيذ منطق مراقبة الاستقرار هنا اذا لزم الأمر
  }
  
  // تحديث الريلاي الأول
  if (newRelay1State != relay1State) {
    relay1State = newRelay1State;
    digitalWrite(RELAY_1_PIN, relay1State ? HIGH : LOW);
    lastRelayChange = currentTime;
    Serial.println("الريلاي 1: " + String(relay1State ? "تشغيل" : "ايقاف"));
  }
  
  // تحديث الريلاي الثاني مع تأخير
  if (newRelay2State != relay2State && (currentTime - lastRelayChange >= RELAY_DELAY_TIME)) {
    relay2State = newRelay2State;
    digitalWrite(RELAY_2_PIN, relay2State ? HIGH : LOW);
    lastRelayChange = currentTime;
    Serial.println("الريلاي 2: " + String(relay2State ? "تشغيل" : "ايقاف"));
  }
}

// ================= دوال مراقبة الشبكة =================
void updateGridStatus() {
  bool currentGridReading = digitalRead(GRID_SENSOR_PIN);
  
  if (currentGridReading != gridAvailable) {
    gridAvailable = currentGridReading;
    gridStable = false;
    gridStableStartTime = millis();
    Serial.println("الشبكة العامة: " + String(gridAvailable ? "متوفرة" : "غير متوفرة"));
  }
  
  // التحقق من استقرار الشبكة
  if (gridAvailable && !gridStable) {
    if (millis() - gridStableStartTime >= GRID_STABILITY_TIME) {
      gridStable = true;
      Serial.println("الشبكة العامة مستقرة");
    }
  } else if (!gridAvailable) {
    gridStable = false;
  }
}

// ================= دوال حفظ واسترجاع الاعدادات =================
void saveSettings() {
  DynamicJsonDocument doc(1024);
  
  // حفظ اعدادات AP
  doc["apSSID"] = apSSID;
  doc["apPassword"] = apPassword;
  doc["apPasswordEnabled"] = apPasswordEnabled;
  
  // حفظ اعدادات النظام
  doc["systemEnabled"] = systemEnabled;
  doc["antiChattering"] = antiChattering;
  doc["relayDelaySeconds"] = relayDelaySeconds;
  
  // حفظ اعدادات الريلايات
  JsonArray relays = doc.createNestedArray("relays");
  for (int i = 0; i < 2; i++) {
    JsonObject relay = relays.createNestedObject();
    relay["onVoltage"] = relaySettings[i].onVoltage;
    relay["offVoltage"] = relaySettings[i].offVoltage;
    relay["manualOverride"] = relaySettings[i].manualOverride;
    relay["manualState"] = relaySettings[i].manualState;
    
    JsonArray timers = relay.createNestedArray("timers");
    for (int j = 0; j < 3; j++) {
      JsonObject timer = timers.createNestedObject();
      timer["enabled"] = relaySettings[i].timers[j].enabled;
      timer["startHour"] = relaySettings[i].timers[j].startHour;
      timer["startMinute"] = relaySettings[i].timers[j].startMinute;
      timer["endHour"] = relaySettings[i].timers[j].endHour;
      timer["endMinute"] = relaySettings[i].timers[j].endMinute;
    }
  }
  
  // حفظ جدول المعايرة
  JsonArray calibration = doc.createNestedArray("calibration");
  for (int i = 0; i < calibrationPointsCount; i++) {
    JsonObject point = calibration.createNestedObject();
    point["adc"] = calibrationTable[i].adcValue;
    point["voltage"] = calibrationTable[i].realVoltage;
  }
  doc["calibrationCount"] = calibrationPointsCount;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // حفظ في EEPROM
  for (int i = 0; i < jsonString.length() && i < EEPROM_SIZE - 1; i++) {
    EEPROM.write(i, jsonString[i]);
  }
  EEPROM.write(jsonString.length(), '\0');
  EEPROM.commit();
  
  Serial.println("تم حفظ الاعدادات");
}

void loadSettings() {
  String jsonString = "";
  char c;
  int i = 0;
  
  // قراءة من EEPROM
  while (i < EEPROM_SIZE - 1) {
    c = EEPROM.read(i);
    if (c == '\0') break;
    jsonString += c;
    i++;
  }
  
  if (jsonString.length() == 0) {
    Serial.println("لا توجد اعدادات محفوظة، استخدام الاعدادات الافتراضية");
    setDefaultSettings();
    return;
  }
  
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    Serial.println("خطأ في قراءة الاعدادات، استخدام الاعدادات الافتراضية");
    setDefaultSettings();
    return;
  }
  
  // استرجاع اعدادات AP
  apSSID = doc["apSSID"] | "Solar System";
  apPassword = doc["apPassword"] | "";
  apPasswordEnabled = doc["apPasswordEnabled"] | false;
  
  // استرجاع اعدادات النظام
  systemEnabled = doc["systemEnabled"] | true;
  antiChattering = doc["antiChattering"] | true;
  relayDelaySeconds = doc["relayDelaySeconds"] | 3;
  
  // استرجاع اعدادات الريلايات
  JsonArray relays = doc["relays"];
  for (int i = 0; i < 2 && i < relays.size(); i++) {
    JsonObject relay = relays[i];
    relaySettings[i].onVoltage = relay["onVoltage"] | (12.0 + i);
    relaySettings[i].offVoltage = relay["offVoltage"] | (11.0 + i);
    relaySettings[i].manualOverride = relay["manualOverride"] | false;
    relaySettings[i].manualState = relay["manualState"] | false;
    
    JsonArray timers = relay["timers"];
    for (int j = 0; j < 3 && j < timers.size(); j++) {
      JsonObject timer = timers[j];
      relaySettings[i].timers[j].enabled = timer["enabled"] | false;
      relaySettings[i].timers[j].startHour = timer["startHour"] | 8;
      relaySettings[i].timers[j].startMinute = timer["startMinute"] | 0;
      relaySettings[i].timers[j].endHour = timer["endHour"] | 17;
      relaySettings[i].timers[j].endMinute = timer["endMinute"] | 0;
    }
  }
  
  // استرجاع جدول المعايرة
  calibrationPointsCount = doc["calibrationCount"] | 12;
  JsonArray calibration = doc["calibration"];
  if (calibration.size() > 0) {
    for (int i = 0; i < calibrationPointsCount && i < calibration.size(); i++) {
      JsonObject point = calibration[i];
      calibrationTable[i].adcValue = point["adc"];
      calibrationTable[i].realVoltage = point["voltage"];
    }
  }
  
  Serial.println("تم تحميل الاعدادات بنجاح");
}

void setDefaultSettings() {
  // اعدادات افتراضية للريلايات
  for (int i = 0; i < 2; i++) {
    relaySettings[i].onVoltage = 12.0 + i;
    relaySettings[i].offVoltage = 11.0 + i;
    relaySettings[i].manualOverride = false;
    relaySettings[i].manualState = false;
    
    // تايمرات افتراضية معطلة
    for (int j = 0; j < 3; j++) {
      relaySettings[i].timers[j].enabled = false;
      relaySettings[i].timers[j].startHour = 8;
      relaySettings[i].timers[j].startMinute = 0;
      relaySettings[i].timers[j].endHour = 17;
      relaySettings[i].timers[j].endMinute = 0;
    }
  }
}

// ================= دوال واجهة الويب =================
String getHTMLHeader() {
  return R"(
<!DOCTYPE html>
<html dir="rtl" lang="ar">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>نظام التحكم بالطاقة الشمسية</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            overflow: hidden;
        }
        .header {
            background: linear-gradient(45deg, #FFA726, #FF7043);
            color: white;
            padding: 20px;
            text-align: center;
        }
        .nav {
            background: #f8f9fa;
            padding: 10px;
            border-bottom: 2px solid #e9ecef;
        }
        .nav-btn {
            display: inline-block;
            padding: 10px 20px;
            margin: 5px;
            background: #007bff;
            color: white;
            text-decoration: none;
            border-radius: 5px;
            transition: background 0.3s;
        }
        .nav-btn:hover {
            background: #0056b3;
        }
        .nav-btn.active {
            background: #28a745;
        }
        .content {
            padding: 20px;
        }
        .card {
            background: #f8f9fa;
            border-radius: 10px;
            padding: 20px;
            margin: 15px 0;
            border-left: 4px solid #007bff;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        .status-card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            text-align: center;
            box-shadow: 0 4px 15px rgba(0,0,0,0.1);
        }
        .voltage-display {
            font-size: 2.5em;
            font-weight: bold;
            color: #007bff;
            margin: 10px 0;
        }
        .relay-status {
            padding: 10px;
            border-radius: 5px;
            font-weight: bold;
            margin: 5px 0;
        }
        .relay-on {
            background: #d4edda;
            color: #155724;
        }
        .relay-off {
            background: #f8d7da;
            color: #721c24;
        }
        .form-group {
            margin: 15px 0;
        }
        .form-group label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
            color: #333;
        }
        .form-control {
            width: 100%;
            padding: 10px;
            border: 2px solid #e9ecef;
            border-radius: 5px;
            font-size: 16px;
            transition: border-color 0.3s;
        }
        .form-control:focus {
            outline: none;
            border-color: #007bff;
        }
        .btn {
            padding: 12px 25px;
            border: none;
            border-radius: 5px;
            font-size: 16px;
            cursor: pointer;
            transition: all 0.3s;
            margin: 5px;
        }
        .btn-primary {
            background: #007bff;
            color: white;
        }
        .btn-primary:hover {
            background: #0056b3;
        }
        .btn-success {
            background: #28a745;
            color: white;
        }
        .btn-success:hover {
            background: #1e7e34;
        }
        .btn-danger {
            background: #dc3545;
            color: white;
        }
        .btn-danger:hover {
            background: #c82333;
        }
        .timer-section {
            border: 2px solid #e9ecef;
            border-radius: 10px;
            padding: 15px;
            margin: 15px 0;
        }
        .timer-header {
            background: #007bff;
            color: white;
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 10px;
        }
        .grid-2 {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        .grid-4 {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 10px;
        }
        @media (max-width: 768px) {
            .grid-2, .grid-4 {
                grid-template-columns: 1fr;
            }
        }
        .alert {
            padding: 15px;
            margin: 15px 0;
            border-radius: 5px;
        }
        .alert-success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .alert-danger {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
    </style>
    <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('voltage').textContent = data.voltage.toFixed(2);
                    document.getElementById('grid-status').textContent = data.gridAvailable ? 'متوفرة' : 'غير متوفرة';
                    document.getElementById('grid-stable').textContent = data.gridStable ? 'مستقرة' : 'غير مستقرة';
                    document.getElementById('relay1-status').textContent = data.relay1 ? 'مشغل' : 'متوقف';
                    document.getElementById('relay1-status').className = 'relay-status ' + (data.relay1 ? 'relay-on' : 'relay-off');
                    document.getElementById('relay2-status').textContent = data.relay2 ? 'مشغل' : 'متوقف';
                    document.getElementById('relay2-status').className = 'relay-status ' + (data.relay2 ? 'relay-on' : 'relay-off');
                })
                .catch(error => console.error('خطأ:', error));
        }
        
        function toggleRelay(relayNum) {
            fetch('/api/relay/' + relayNum + '/toggle', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        updateStatus();
                    } else {
                        alert('فشل في تغيير حالة الريلاي');
                    }
                });
        }
        
        setInterval(updateStatus, 2000);
        document.addEventListener('DOMContentLoaded', updateStatus);
    </script>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>نظام التحكم بأحمال الطاقة الشمسية</h1>
            <p>التحكم الذكي والمراقبة المتقدمة</p>
        </div>
        <div class="nav">
            <a href="/" class="nav-btn">المراقبة</a>
            <a href="/settings" class="nav-btn">الاعدادات</a>
            <a href="/calibration" class="nav-btn">المعايرة</a>
            <a href="/timers" class="nav-btn">التايمرات</a>
            <a href="/system" class="nav-btn">النظام</a>
        </div>
        <div class="content">
)";
}

String getHTMLFooter() {
  return R"(
        </div>
    </div>
</body>
</html>
)";
}

// ================= صفحات الويب =================
void handleRoot() {
  String html = getHTMLHeader();
  
  html += R"(
<div class="status-grid">
    <div class="status-card">
        <h3>جهد البطارية</h3>
        <div class="voltage-display" id="voltage">)" + String(currentVoltage, 2) + R"(</div>
        <p>فولت</p>
    </div>
    <div class="status-card">
        <h3>الشبكة العامة</h3>
        <p><strong>الحالة:</strong> <span id="grid-status">)" + String(gridAvailable ? "متوفرة" : "غير متوفرة") + R"(</span></p>
        <p><strong>الاستقرار:</strong> <span id="grid-stable">)" + String(gridStable ? "مستقرة" : "غير مستقرة") + R"(</span></p>
    </div>
    <div class="status-card">
        <h3>الريلاي الأول</h3>
        <div class="relay-status )" + String(relay1State ? "relay-on" : "relay-off") + R"(" id="relay1-status">)" + 
        String(relay1State ? "مشغل" : "متوقف") + R"(</div>
        <button onclick="toggleRelay(1)" class="btn btn-primary">تبديل الحالة</button>
    </div>
    <div class="status-card">
        <h3>الريلاي الثاني</h3>
        <div class="relay-status )" + String(relay2State ? "relay-on" : "relay-off") + R"(" id="relay2-status">)" + 
        String(relay2State ? "مشغل" : "متوقف") + R"(</div>
        <button onclick="toggleRelay(2)" class="btn btn-primary">تبديل الحالة</button>
    </div>
</div>

<div class="card">
    <h3>معلومات النظام</h3>
    <div class="grid-2">
        <div>
            <p><strong>وقت التشغيل:</strong> )" + String(millis() / 1000) + R"( ثانية</p>
            <p><strong>الذاكرة المتاحة:</strong> )" + String(ESP.getFreeHeap()) + R"( بايت</p>
        </div>
        <div>
            <p><strong>قوة الاشارة:</strong> )" + String(WiFi.RSSI()) + R"( dBm</p>
            <p><strong>عنوان IP:</strong> )" + WiFi.softAPIP().toString() + R"(</p>
        </div>
    </div>
</div>
)";
  
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}

void handleSettings() {
  if (server.method() == HTTP_POST) {
    // معالجة تحديث الاعدادات
    for (int i = 0; i < 2; i++) {
      relaySettings[i].onVoltage = server.arg("relay" + String(i+1) + "_on_voltage").toFloat();
      relaySettings[i].offVoltage = server.arg("relay" + String(i+1) + "_off_voltage").toFloat();
      relaySettings[i].manualOverride = server.hasArg("relay" + String(i+1) + "_manual");
      if (relaySettings[i].manualOverride) {
        relaySettings[i].manualState = server.hasArg("relay" + String(i+1) + "_manual_state");
      }
    }
    
    systemEnabled = server.hasArg("system_enabled");
    antiChattering = server.hasArg("anti_chattering");
    relayDelaySeconds = server.arg("relay_delay").toInt();
    
    if (server.hasArg("ap_ssid")) {
      apSSID = server.arg("ap_ssid");
    }
    if (server.hasArg("ap_password")) {
      apPassword = server.arg("ap_password");
      apPasswordEnabled = (apPassword.length() > 0);
    }
    
    saveSettings();
    
    // اعادة تشغيل AP اذا تغيرت الاعدادات
    WiFi.softAPdisconnect();
    delay(100);
    if (apPasswordEnabled) {
      WiFi.softAP(apSSID.c_str(), apPassword.c_str());
    } else {
      WiFi.softAP(apSSID.c_str());
    }
  }
  
  String html = getHTMLHeader();
  
  html += R"(<h2>اعدادات النظام</h2>)";
  
  if (server.method() == HTTP_POST) {
    html += R"(<div class="alert alert-success">تم حفظ الاعدادات بنجاح!</div>)";
  }
  
  html += R"(
<form method="post">
    <div class="card">
        <h3>اعدادات نقطة الوصول</h3>
        <div class="form-group">
            <label>اسم الشبكة (SSID):</label>
            <input type="text" name="ap_ssid" class="form-control" value=")" + apSSID + R"(" required>
        </div>
        <div class="form-group">
            <label>كلمة المرور (اتركها فارغة لالغاء الحماية):</label>
            <input type="password" name="ap_password" class="form-control" value=")" + apPassword + R"(">
        </div>
    </div>
    
    <div class="card">
        <h3>اعدادات عامة للنظام</h3>
        <div class="form-group">
            <label>
                <input type="checkbox" name="system_enabled" )" + String(systemEnabled ? "checked" : "") + R"(> 
                تفعيل النظام
            </label>
        </div>
        <div class="form-group">
            <label>
                <input type="checkbox" name="anti_chattering" )" + String(antiChattering ? "checked" : "") + R"(> 
                منع تكرار الاقلاع
            </label>
        </div>
        <div class="form-group">
            <label>زمن التأخير بين الريلايات (ثواني):</label>
            <input type="number" name="relay_delay" class="form-control" value=")" + String(relayDelaySeconds) + R"(" min="1" max="60">
        </div>
    </div>
)";

  // اعدادات الريلايات
  for (int i = 0; i < 2; i++) {
    html += R"(
    <div class="card">
        <h3>اعدادات الريلاي )" + String(i+1) + R"(</h3>
        <div class="grid-2">
            <div class="form-group">
                <label>جهد التشغيل (فولت):</label>
                <input type="number" step="0.1" name="relay)" + String(i+1) + R"(_on_voltage" class="form-control" 
                       value=")" + String(relaySettings[i].onVoltage, 1) + R"(" required>
            </div>
            <div class="form-group">
                <label>جهد الايقاف (فولت):</label>
                <input type="number" step="0.1" name="relay)" + String(i+1) + R"(_off_voltage" class="form-control" 
                       value=")" + String(relaySettings[i].offVoltage, 1) + R"(" required>
            </div>
        </div>
        <div class="form-group">
            <label>
                <input type="checkbox" name="relay)" + String(i+1) + R"(_manual" )" + 
                String(relaySettings[i].manualOverride ? "checked" : "") + R"(> 
                التحكم اليدوي
            </label>
        </div>
        <div class="form-group">
            <label>
                <input type="checkbox" name="relay)" + String(i+1) + R"(_manual_state" )" + 
                String(relaySettings[i].manualState ? "checked" : "") + R"(> 
                حالة التحكم اليدوي (مشغل)
            </label>
        </div>
    </div>
)";
  }
  
  html += R"(
    <div style="text-align: center; margin: 20px 0;">
        <button type="submit" class="btn btn-success">حفظ الاعدادات</button>
    </div>
</form>
)";
  
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}

// دالة صفحة API للحالة
void handleAPIStatus() {
  DynamicJsonDocument doc(512);
  
  doc["voltage"] = currentVoltage;
  doc["gridAvailable"] = gridAvailable;
  doc["gridStable"] = gridStable;
  doc["relay1"] = relay1State;
  doc["relay2"] = relay2State;
  doc["systemEnabled"] = systemEnabled;
  doc["uptime"] = millis() / 1000;
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["rssi"] = WiFi.RSSI();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// دالة تبديل حالة الريلاي
void handleRelayToggle() {
  int relayNum = server.pathArg(0).toInt();
  
  if (relayNum >= 1 && relayNum <= 2) {
    int index = relayNum - 1;
    relaySettings[index].manualOverride = true;
    relaySettings[index].manualState = !relaySettings[index].manualState;
    
    // تطبيق التغيير فوراً
    if (relayNum == 1) {
      relay1State = relaySettings[index].manualState;
      digitalWrite(RELAY_1_PIN, relay1State ? HIGH : LOW);
    } else {
      relay2State = relaySettings[index].manualState;
      digitalWrite(RELAY_2_PIN, relay2State ? HIGH : LOW);
    }
    
    saveSettings();
    
    DynamicJsonDocument doc(128);
    doc["success"] = true;
    doc["relayState"] = relaySettings[index].manualState;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  } else {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid relay number\"}");
  }
}

// ================= الاعداد الاولي =================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("بدء تشغيل نظام التحكم بأحمال الطاقة الشمسية");
  
  // تهيئة EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // تهيئة المداخل والمخارج
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(GRID_SENSOR_PIN, INPUT);
  
  digitalWrite(RELAY_1_PIN, LOW);
  digitalWrite(RELAY_2_PIN, LOW);
  
  // تحميل الاعدادات
  loadSettings();
  
  // تهيئة نقطة الوصول
  WiFi.mode(WIFI_AP);
  if (apPasswordEnabled && apPassword.length() >= 8) {
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  } else {
    WiFi.softAP(apSSID.c_str());
  }
  
  Serial.print("نقطة الوصول: ");
  Serial.println(apSSID);
  Serial.print("عنوان IP: ");
  Serial.println(WiFi.softAPIP());
  
  // تهيئة التوقيت
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  
  // تهيئة خادم الويب
  server.on("/", handleRoot);
  server.on("/settings", handleSettings);
  server.on("/api/status", handleAPIStatus);
  server.on("/api/relay/1/toggle", HTTP_POST, handleRelayToggle);
  server.on("/api/relay/2/toggle", HTTP_POST, handleRelayToggle);
  
  server.begin();
  Serial.println("خادم الويب جاهز!");
  
  Serial.println("النظام جاهز للعمل");
}

// ================= الحلقة الرئيسية =================
void loop() {
  server.handleClient();
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // قراءة وتحديث جهد البطارية
    float adcReading = takeAverageADCReading();
    currentVoltage = getCorrectedVoltage(adcReading);
    
    // تحديث حالة الشبكة العامة
    updateGridStatus();
    
    // تحديث حالة الريلايات
    updateRelayStates();
    
    // طباعة الحالة كل 10 ثواني
    if (currentMillis % 10000 < interval) {
      Serial.println("=== حالة النظام ===");
      Serial.println("جهد البطارية: " + String(currentVoltage, 2) + " فولت");
      Serial.println("الشبكة العامة: " + String(gridAvailable ? "متوفرة" : "غير متوفرة"));
      Serial.println("استقرار الشبكة: " + String(gridStable ? "مستقرة" : "غير مستقرة"));
      Serial.println("الريلاي 1: " + String(relay1State ? "مشغل" : "متوقف"));
      Serial.println("الريلاي 2: " + String(relay2State ? "مشغل" : "متوقف"));
      Serial.println("==================");
    }
  }
}