#pragma once
#include <stdbool.h>
#include <time.h>
void clock_service_init(void);
bool clock_service_read(struct tm *local);
bool clock_service_configured(void);
