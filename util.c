#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} DateTime;
extern uint8_t seg7[];
extern uint8_t isFlowing;
extern uint8_t flow_index;
extern uint8_t flow_direction;
extern DateTime currentDateTime;
extern void UARTStringPut(const char * cp);
uint8_t getDaysInMonth(uint16_t year, uint8_t month) {
    if (month == 2) {
        // 闰年判断
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
            return 29; // 闰年2月
        } else {
            return 28; // 平年2月
        }
    } else if (month == 4 || month == 6 || month == 9 || month == 11) {
        return 30; // 小月
    } else {
        return 31; // 大月
    }
}

uint8_t CharToSeg7(unsigned char c)
{
    if (c >= '0' && c <= '9')
        return seg7[c - '0'];
    else if (c >= 'A' && c <= 'F')
        return seg7[c - 'A' + 10];
    else if (c >= 'a' && c <= 'f')
        return seg7[c - 'a' + 10];
    else if (c == 'Y' || c == 'y')
        return 0x6e;
    else if (c == 'L' || c == 'l')
        return 0x38;
    else if (c == 'V' || c == 'v')
        return 0x3e;
    else if (c == '-')
        return 0x40; 
    else if (c == ':')
        return 0x80;
    else
        return 0x00; 
}

void StringToSeg7(const char *str, uint8_t *data)
{
	uint8_t i = 0;
    uint8_t extra_len = 0;
    for (i = 0; i < 8; ) {
        if (str[i + extra_len] == '\0') {
            data[i] = 0x00; 
             // 如果字符串结束，填充剩余部分为0
            for (; i < 8; i++) {
                data[i] = 0x00;
            }
            return;
        }else if (str[i + extra_len] == '-' || str[i + extra_len] == ':') {
            data[(i+1)%8] |= 0x80;
            extra_len++;
        }
        else {
            data[i] = CharToSeg7(str[i + extra_len]);
            i++;
        }
    }
}
void FlowingStringToSeg7(const char *str, uint8_t *data, uint8_t datalen)
{
    uint8_t i;
    uint8_t extra_len = 0;
    for (i = 0; i < 8;) {
        if (str[(flow_index+i+extra_len)%datalen] == '\0') {
            data[i] = 0x00; 
            i++;
        }else if (str[(flow_index+i+extra_len)%datalen] == '-' || str[(flow_index+i+extra_len)%datalen] == ':') {
						data[(i+7)%8] |= 0x80; 
						extra_len++;
        } 
        else {
            data[i] = CharToSeg7(str[(flow_index+i+extra_len)%datalen]);
            i++;
        }
    }
}
// 解析时间字符串
DateTime parseTime(const char *timeStr) {
    DateTime datetime;
		sscanf(timeStr, "%hhu%*[:-]%hhu%*[:-]%hhu", &datetime.hour, &datetime.minute, &datetime.second);
    return datetime;
}
// 格式化时间为字符串
void formatTime(char *buffer, const DateTime *datetime) {
    sprintf(buffer, "%02d:%02d:%02d", datetime->hour, datetime->minute, datetime->second);
}

void formatDate(char *buffer, const DateTime *datetime) {
    sprintf(buffer, "%04d-%02d-%02d", datetime->year, datetime->month, datetime->day);
}

void formatDateTime(char *buffer, const DateTime *datetime) {
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d ", datetime->year, datetime->month, datetime->day, datetime->hour, datetime->minute, datetime->second);
}
// 更新时间并处理进位
void updateTime(DateTime *datetime) {
    if (datetime->second >= 60) {
        datetime->second = 0;
        datetime->minute++;
    }
    if (datetime->minute >= 60) {
        datetime->minute = 0;
        datetime->hour++;
    }
    if (datetime->hour >= 24) {
        datetime->hour = 0;
        datetime->day++;
    }
    if (datetime->day >  getDaysInMonth(datetime->year, datetime->month)) {
        datetime->day = 1;
        datetime->month++;
    }
    if (datetime->month > 12) {
        datetime->month = 1;
        datetime->year++;
    }
}
// Helper function to convert a string to uppercase
void to_upper(char *str) {
	uint8_t i;
    for (i = 0; str[i]; i++) {
        if (str[i] >= 'a' && str[i] <= 'z') {
            str[i] -= 32; // Convert to uppercase
        }
    }
}
void makeDateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, DateTime *datetime) {
    datetime->year = year;
    datetime->month = month;
    datetime->day = day;
    datetime->hour = hour;
    datetime->minute = minute;
    datetime->second = second;
}

