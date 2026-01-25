import pigpio
import time
import sys
import termios
import tty
import select

# ============================================================================
# КОНФИГУРАЦИЯ - МОЖНО МЕНЯТЬ ЗДЕСЬ!
# ============================================================================

# Пиновая конфигурация
RIGHT_PWM_PIN = 13     # GPIO13 (PWM1)
RIGHT_IN1_PIN = 19     # GPIO19 (IN3 на L298N)
RIGHT_IN2_PIN = 26     # GPIO26 (IN4 на L298N)

LEFT_PWM_PIN = 18      # GPIO18 (PWM0)
LEFT_IN1_PIN = 20      # GPIO20 (IN1 на L298N)
LEFT_IN2_PIN = 21      # GPIO21 (IN2 на L298N)

RIGHT_ENC_A = 5        # GPIO5 (S1 правого мотора)
RIGHT_ENC_B = 6        # GPIO6 (S2 правого мотора)
LEFT_ENC_A = 17        # GPIO17 (S1 левого мотора)
LEFT_ENC_B = 27        # GPIO27 (S2 левого мотора)

# НАСТРОЙКИ ШИМ
PWM_FREQUENCY = 400    # Частота ШИМ в Гц
MAX_PWM = 85           # Максимальный ШИМ в % (ограничиваем ток)
MIN_PWM = 30           # Минимальный рабочий ШИМ
START_PWM = 30         # Стартовый ШИМ ?


# Настройки плавного старта
SMOOTH_STEP = 5        # Шаг изменения скорости для плавности
SMOOTH_DELAY = 0.05    # Задержка между шагами

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ PIGPIO
# ============================================================================

pi = pigpio.pi()
if not pi.connected:
    print("Ошибка: Не удалось подключиться к pigpio демону")
    print("Запустите: sudo pigpiod")
    sys.exit(1)


# ============================================================================
# ПРОСТОЙ КЛАСС ЭНКОДЕРА (ТОЛЬКО ПОДСЧЁТ ИМПУЛЬСОВ)
# ============================================================================

class EncoderCounter:
    def __init__(self, pin_a, name="Encoder"):
        self.pin_a = pin_a
        self.name = name
        self.__count = 0
        
        # Настройка пина
        pi.set_mode(pin_a, pigpio.INPUT)
        pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        
        # Callback только на один пин для простоты
        self.cb = pi.callback(pin_a, pigpio.EITHER_EDGE, self._count_callback)
    
    def _count_callback(self, gpio, level, tick):
        self.__count += 1
    
    def get_count(self):
        return self.__count
    
    def reset(self):
        self.__count = 0
    
    def cleanup(self):
        if hasattr(self, 'cb'):
            self.cb.cancel()


# ============================================================================
# ПРОСТОЙ И НАДЁЖНЫЙ КЛАСС МОТОРА
# ============================================================================

class Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin, name="Motor"):
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.name = name
        self.current_pwm = 0
        
        # Настройка пинов
        pi.set_mode(pwm_pin, pigpio.OUTPUT)
        pi.set_mode(in1_pin, pigpio.OUTPUT)
        pi.set_mode(in2_pin, pigpio.OUTPUT)
        
        # Инициализация ШИМ
        pi.set_PWM_frequency(pwm_pin, PWM_FREQUENCY)
        pi.set_PWM_range(pwm_pin, MAX_PWM)
        pi.set_PWM_dutycycle(pwm_pin, 0)
        
        # Установка направления
        pi.write(in1_pin, 0)
        pi.write(in2_pin, 0)
    
    def _apply_pwm_direct(self, pwm):
        """Непосредственное применение скорости"""
        # Ограничиваем скорость
        pwm = max(-MAX_PWM, min(MAX_PWM, pwm))
        self.current_pwm = pwm
        
        # Управление направлением
        if pwm > 0:
            # ВПЕРЁД
            pi.write(self.in1_pin, 1)
            pi.write(self.in2_pin, 0)
            pwm_value = max(MIN_PWM, pwm)
        elif pwm < 0:
            # НАЗАД
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 1)
            pwm_value = max(MIN_PWM, -pwm)
        else:
            # СТОП
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 0)
            pwm_value = 0
        
        # Установка ШИМ
        pi.set_PWM_dutycycle(self.pwm_pin, pwm_value)
        
        return pwm_value
    
    def set_pwm_smooth(self, target_pwm):
        """Плавная установка заполнения PWM"""
        current = self.current_pwm
        target = target_pwm
        
        if current == target:
            return
        
        # Определяем направление и количество шагов
        if target > current:
            step = SMOOTH_STEP
        else:
            step = -SMOOTH_STEP
        
        # Вычисляем количество шагов
        steps = abs(current - target) // abs(step)
        
        # # Плавное изменение
        # for i in range(steps):
        #     new_pwm = current + step
        #     self._apply_pwm_direct(new_pwm)
        #     time.sleep(SMOOTH_DELAY) #      задержку без delay? !!!!!
        
        # Финальная установка точного значения
        self._apply_pwm_direct(target)
    
   
    
    def stop(self):
        """Остановка"""
        self._apply_pwm_direct(0)
        
    
    def brake(self):
        """Торможение"""
        pi.write(self.in1_pin, 1)
        pi.write(self.in2_pin, 1)
        pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.current_speed = 0
        print(f"{self.name}: ТОРМОЖЕНИЕ")

    # def cleanup(self):
    #     pi.stop()
