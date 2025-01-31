// #include "tasks.h"
// #include <Preferences.h>
// #include <TimeLib.h>
// #include <Arduino.h>


// Preferences nvs;

// void createEntry(const char* date, const String& message) {
//     nvs.begin("calendar", false);
//     nvs.putString(date, message);
//     // nvs.end();
//     // Serial.println("Запись создана: " + date + " - " + message);
// }

// // Функция для создания повторяющейся записи
// void createRecurringEntry(const char* startDate, const String& message, int intervalDays) {
//     time_t currentTime = now();
//     struct tm tmr;
//     tmElements_t tm;
//     strptime(startDate, "%Y-%m-%d", &tmr);
//     tm.Day = tmr.tm_mday;
//     tm.Month = tmr.tm_mon;
//     tm.Year = tmr.tm_year;
//     time_t startTime = makeTime(tm);

//     while (startTime <= currentTime + SECS_PER_DAY * 365) { // На год вперед
//         String dateStr = String(year(startTime)) + "-" + String(month(startTime)) + "-" + String(day(startTime));
//         createEntry(dateStr.c_str(), message);
//         startTime += intervalDays * SECS_PER_DAY;
//     }
// }

// // Функция для получения всех записей на текущий день
// void getEntriesForDate(const char* date) {
//     nvs.begin("calendar", true);
//     String message = nvs.getString(date, "");
//     nvs.end();
//     if (message != "") {
//         // Serial.println("Запись на " + date + ": " + message);
//     } else {
//         // Serial.println("Записей на " + date + " нет.");
//     }
// }

// // Функция для удаления всех записей на определенный день
// void deleteEntriesForDate(const char* date) {
//     nvs.begin("calendar", false);
//     if (nvs.remove(date)) {
//         // Serial.println("Записи на " + date + " удалены.");
//     } else {
//         // Serial.println("Записей на " + date + " нет.");
//     }
//     nvs.end();
// }