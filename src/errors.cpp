#include <errors.h>

void Error_Handler() {
    __disable_irq();
    for (;;) {
    }
}
extern "C" void assert_failed(uint8_t* file, uint32_t line) {
    for (;;) {
    }
}