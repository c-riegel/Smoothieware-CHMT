#include "gpio.h"

#include "stm32f4xx.h"

extern "C" uint32_t Set_GPIO_Clock(uint32_t);

static GPIO_TypeDef* const gpios[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

GPIO::GPIO(PinName pin) {
    this->port = STM_PORT(pin);
    this->pin = STM_PIN(pin);

    setup();
}

GPIO::GPIO(uint8_t port, uint8_t pin) {
	GPIO::port = port;
	GPIO::pin = pin;

	setup();
}

GPIO::GPIO(uint8_t port, uint8_t pin, uint8_t direction) {
	GPIO::port = port;
	GPIO::pin = pin;

	setup();

	set_direction(direction);
}
// GPIO::~GPIO() {}

void GPIO::setup() {
    Set_GPIO_Clock(port);

    gpios[port]->OTYPER &= ~(1 << pin);
    gpios[port]->PUPDR &= ~(0x3 << (2*pin));
}

void GPIO::set_direction(uint8_t direction) {
    if (direction)
        output();
    else
        input();
}

void GPIO::output() {
    gpios[port]->MODER = (gpios[port]->MODER & ~(0x3<<(2*pin))) | (0x1<<(2*pin));
}

void GPIO::input() {
    gpios[port]->MODER &= ~(0x3<<(2*pin));
}

void GPIO::write(uint8_t value) {
	output();
	if (value)
		set();
	else
		clear();
}

void GPIO::set() {
    gpios[port]->BSRR = (0x1 << pin);
}

void GPIO::clear() {
    gpios[port]->BSRR = ((0x1 << 16) << pin);
}

uint8_t GPIO::get() {
	return (gpios[port]->IDR & (1UL << pin)) ? 0xFF : 0;
}

int GPIO::operator=(int value) {
    if (value)
        set();
    else
        clear();
    return value;
}
