#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "logger.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#endif

// IMU driver interface
class IMUInterface {
public:
    enum DLPF {
        DLPF_OFF,
        DLPF_MAX,
        DLPF_100HZ_APPROX,
        DLPF_50HZ_APPROX,
        DLPF_5HZ_APPROX,
        DLPF_MIN
    };
    enum AccelRange {
        ACCEL_RANGE_MIN,
        ACCEL_RANGE_2G,
        ACCEL_RANGE_4G,
        ACCEL_RANGE_8G,
        ACCEL_RANGE_16G,
        ACCEL_RANGE_MAX
    };
    enum GyroRange {
        GYRO_RANGE_MIN,
        GYRO_RANGE_250DPS,
        GYRO_RANGE_500DPS,
        GYRO_RANGE_1000DPS,
        GYRO_RANGE_2000DPS,
        GYRO_RANGE_MAX
    };
    enum Rate {
        RATE_MIN,
        RATE_50HZ_APPROX,
        RATE_1KHZ_APPROX,
        RATE_8KHZ_APPROX,
        RATE_MAX
    };
    virtual bool begin() = 0;
    virtual void reset() = 0;
    virtual int status() const = 0; // 0 - success, otherwise error
    virtual uint8_t whoAmI() = 0;
    virtual bool read() = 0;
    virtual void waitForData() = 0;
    virtual void getAccel(float& x, float& y, float& z) const = 0;
    virtual void getGyro(float& x, float& y, float& z) const = 0;
    virtual void getMag(float& x, float& y, float& z) const = 0;
    virtual float getTemp() = 0;
    virtual bool setRate(const Rate rate) = 0;
    virtual float getRate() = 0;
    virtual bool setAccelRange(const AccelRange range) = 0;
    virtual bool setGyroRange(const GyroRange range) = 0;
    virtual bool setDLPF(const DLPF dlpf) = 0;
    virtual const char* getModel() const = 0;
    virtual bool setupInterrupt(int pin = -1) = 0;
};

// Base for all IMU drivers
class IMUBase : public IMUInterface, public Logger {
private:
    bool usingInterrupt = false;
    int interruptPin = -1;

#ifdef ESP32
    SemaphoreHandle_t interruptSemaphore = NULL;
#endif

    // Статический метод-обертка для прерывания
    static void ARDUINO_ISR_ATTR interruptHandler() {
        IMUBase::handleInterruptStatic();
    }

    // Статический метод для обработки прерывания
    static void handleInterruptStatic() {
#ifdef ESP32
        // В этой упрощенной версии мы просто читаем данные
        // Для многопоточной работы потребуется более сложная логика
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Здесь мы не можем обратиться к конкретному экземпляру,
        // поэтому просто даем семафор если он существует
        // В реальном использовании нужно будет переопределить этот метод
        // в конкретных классах IMU
        if (false) { // заглушка
            xSemaphoreGiveFromISR(nullptr, &xHigherPriorityTaskWoken);
        }
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
#endif
    }

    bool setupInterruptPin(uint8_t pin) {
        // В этой базовой реализации прерывания не поддерживаются
        // Конкретные классы IMU должны переопределить этот метод
        log("Base interrupt setup not supported - override in derived class");
        return false;
    }

public:
    virtual ~IMUBase() {
        if (interruptPin != -1) {
            detachInterrupt(digitalPinToInterrupt(interruptPin));
            interruptPin = -1;
        }

#ifdef ESP32
        if (interruptSemaphore != NULL) {
            vSemaphoreDelete(interruptSemaphore);
            interruptSemaphore = NULL;
        }
#endif
        usingInterrupt = false;
    }

protected:
    bool setupInterrupt(int pin = -1) override {
        if (usingInterrupt) {
            log("Interrupt already configured");
            return true;
        }

        if (pin == -1) {
            log("Timer interrupts not available, please specify a pin number");
            return false;
        } else {
            return setupInterruptPin(pin);
        }
    }

    // Защищенные методы для использования в производных классах
#ifdef ESP32
protected:
    bool createInterruptSemaphore() {
        if (interruptSemaphore == NULL) {
            interruptSemaphore = xSemaphoreCreateBinary();
            return (interruptSemaphore != NULL);
        }
        return true;
    }

    void giveInterruptSemaphore() {
        if (interruptSemaphore != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(interruptSemaphore, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    }

    void setupPinInterrupt(uint8_t pin) {
        pinMode(pin, INPUT_PULLUP);
        detachInterrupt(digitalPinToInterrupt(pin));
        attachInterrupt(digitalPinToInterrupt(pin), IMUBase::interruptHandler, FALLING);
        interruptPin = pin;
        usingInterrupt = true;
    }
#endif

public:
    void waitForData() override {
        if (this->status() != 0) return; // don't wait if there's an error

        if (usingInterrupt) {
#ifdef ESP32
            if (interruptSemaphore != NULL) {
                xSemaphoreTake(interruptSemaphore, portMAX_DELAY); // wait using interrupt
                this->read();
            } else {
                // Fallback to polling if semaphore is not available
                while (!this->read()) {
                    delay(1);
                }
            }
#endif
        } else {
            // Polling mode
            while (!this->read()) {
                delay(1);
            }
        }
    }

    bool isUsingInterrupt() const {
        return usingInterrupt;
    }

    int getInterruptPin() const {
        return interruptPin;
    }
};