#include "main.h"
#include <protocols/wireless.h>

osMessageQueueId_t __wireless_get_queue(void) {
    return command_queue;
}