#!/usr/bin/env python3
"""
ИСПРАВЛЕННЫЙ тестовый скрипт для робота
- Увеличен минимальный ШИМ для L298N
- Понижена частота ШИМ для устранения писка
- Исправлено направление движения
"""

import pigpio
import time
import threading
import sys

# ============================================================================
# КОНФИГУРАЦИЯ ПИНОВ
# ============================================================================

# Правый мотор
RIGHT_PWM_PIN = 13     # GPIO13 (PWM1)
RIGHT_IN1_PIN = 19     # GPIO19 (IN3 на L298N)
RIGHT_IN2_PIN = 26     # GPIO26 (IN4 на L298N)

# Левый мотор  
LEFT_PWM_PIN = 18      # GPIO18 (PWM0)
LEFT_IN1_PIN = 20      # GPIO20 (IN1 на L298N)
LEFT_IN2_PIN = 21      # GPIO21 (IN2 на L298N)

# Энкодеры
RIGHT_ENC_A = 5        # GPIO5 (S1 правого мотора)
RIGHT_ENC_B = 6        # GPIO6 (S2 правого мотора)
LEFT_ENC_A = 17        # GPIO17 (S1 левого мотора)
LEFT_ENC_B = 27        # GPIO27 (S2 левого мотора)

MAX_PWM = 45 # При 50 срабатывает защита на АКБ

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ PIGPIO
# ============================================================================

pi = pigpio.pi()
if not pi.connected:
    print("Ошибка: Не удалось подключиться к pigpio демону")
    print("Запустите: sudo pigpiod")
    sys.exit(1)

# ============================================================================
# ИСПРАВЛЕННЫЙ КЛАСС ДЛЯ УПРАВЛЕНИЯ МОТОРОМ
# ============================================================================

class Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin, name="Motor"):
        global MAX_PWM
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.name = name
        self.speed = 0  # 0-100%
        
        # НАСТРОЙКИ L298N
        self.MIN_PWM = 10  # Минимальный ШИМ для L298N (мёртвая зона)
        #self.MAX_PWM = 60 # При 70 срабатывает защита на АКБ
        self.PWM_FREQ = 450  # Частота ШИМ - УМЕНЬШЕНА для устранения писка
        
        # Настройка пинов
        pi.set_mode(pwm_pin, pigpio.OUTPUT)
        pi.set_mode(in1_pin, pigpio.OUTPUT)
        pi.set_mode(in2_pin, pigpio.OUTPUT)
        
        # Инициализация ШИМ с пониженной частотой
        pi.set_PWM_frequency(pwm_pin, self.PWM_FREQ)  # Уменьшенная частота!
        pi.set_PWM_range(pwm_pin, MAX_PWM)
        pi.set_PWM_dutycycle(pwm_pin, 0)
        
        # Установка направления
        pi.write(in1_pin, 0)
        pi.write(in2_pin, 0)
        
        print(f"{name} инициализирован:")
        print(f"  PWM={pwm_pin} (частота {self.PWM_FREQ}Гц)")
        print(f"  IN1={in1_pin}, IN2={in2_pin}")
        print(f"  Мин.ШИМ: {self.MIN_PWM}%")
    
    def _pwm_with_minimum(self, speed_percent):
        """Преобразование скорости с учётом минимального ШИМ"""
        if speed_percent == 0:
            return 0
        
        # Применяем минимальный порог для L298N
        if 0 < abs(speed_percent) < self.MIN_PWM:
            # Для низких скоростей используем мин.ШИМ
            return self.MIN_PWM if speed_percent > 0 else -self.MIN_PWM
        
        return speed_percent
    
    def set_speed(self, speed_percent, immediate=False):
        """Установка скорости от -MAX_PWM до MAX_PWM"""
        # Ограничиваем скорость
        speed_percent = max(-MAX_PWM, min(MAX_PWM, speed_percent))
        
        # Применяем минимальный ШИМ
        actual_speed = self._pwm_with_minimum(speed_percent)
        self.speed = speed_percent  # Сохраняем оригинальное значение для отображения
        
        # Управление направлением (ИСПРАВЛЕННАЯ ЛОГИКА)
        if actual_speed > 0:
            # ВПЕРЁД: IN1=1, IN2=0
            pi.write(self.in1_pin, 1)
            pi.write(self.in2_pin, 0)
            pwm_value = actual_speed
            direction = "ВПЕРЁД"
        elif actual_speed < 0:
            # НАЗАД: IN1=0, IN2=1
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 1)
            pwm_value = -actual_speed
            direction = "НАЗАД"
        else:
            # СТОП: IN1=0, IN2=0
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 0)
            pwm_value = 0
            direction = "СТОП"
        
        # Установка ШИМ
        pi.set_PWM_dutycycle(self.pwm_pin, pwm_value)
        
        # Вывод отладки (только при изменении)
        if hasattr(self, '_last_debug') and self._last_debug == (direction, pwm_value):
            return
        self._last_debug = (direction, pwm_value)
        
        print(f"{self.name}: {direction} {pwm_value}% (запрошено {speed_percent}%)")
        return speed_percent
    
    def stop(self):
        """Полная остановка"""
        self.set_speed(0)
    
    def brake(self):
        """Торможение (короткое замыкание обмоток)"""
        pi.write(self.in1_pin, 1)
        pi.write(self.in2_pin, 1)
        pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.speed = 0
        print(f"{self.name}: ТОРМОЖЕНИЕ")

# ============================================================================
# ПРОСТОЙ КЛАСС ЭНКОДЕРА (без сложных вычислений)
# ============================================================================

class SimpleEncoder:
    def __init__(self, pin_a, pin_b, name="Encoder"):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.count = 0
        self.last_a = pi.read(pin_a)
        self.last_b = pi.read(pin_b)
        
        # Настройка пинов
        pi.set_mode(pin_a, pigpio.INPUT)
        pi.set_mode(pin_b, pigpio.INPUT)
        pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Callback
        self.cb_a = pi.callback(pin_a, pigpio.EITHER_EDGE, self._callback)
        
        print(f"{name} на пинах A={pin_a}, B={pin_b}")
    
    def _callback(self, gpio, level, tick):
        self.count += 1
    
    def get_count(self):
        return self.count
    
    def reset(self):
        self.count = 0
    
    def cleanup(self):
        if hasattr(self, 'cb_a'):
            self.cb_a.cancel()

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ
# ============================================================================

print("=" * 60)
print("🤖 ИСПРАВЛЕННЫЙ ТЕСТ РОБОТА")
print("=" * 60)

# Создаём моторы
left_motor = Motor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, "Левый мотор")
right_motor = Motor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, "Правый мотор")

# Создаём энкодеры
encoder_left = SimpleEncoder(LEFT_ENC_A, LEFT_ENC_B, "Левый энкодер")
encoder_right = SimpleEncoder(RIGHT_ENC_A, RIGHT_ENC_B, "Правый энкодер")

print("\n✅ Система инициализирована")
time.sleep(1)

# ============================================================================
# ТЕСТ НАПРАВЛЕНИЙ - ВАЖНО: ПРОВЕРЬТЕ ЭТО СНАЧАЛА!
# ============================================================================

def test_directions():
    """Тест направлений движения - САМЫЙ ВАЖНЫЙ ТЕСТ!"""
    print("\n" + "=" * 60)
    print("🧭 ТЕСТ НАПРАВЛЕНИЙ ДВИЖЕНИЯ")
    print("=" * 60)
    print("Поднимите робота, чтобы колёса не касались поверхности!")
    print("Наблюдайте за направлением вращения колёс.")
    input("Нажмите Enter для начала теста...")
    
    tests = [
        ("ЛЕВОЕ колесо ВПЕРЁД", lambda: left_motor.set_speed(40), "Должно вращаться ВПЕРЁД"),
        ("ЛЕВОЕ колево НАЗАД", lambda: left_motor.set_speed(-40), "Должно вращаться НАЗАД"),
        ("ПРАВОЕ колесо ВПЕРЁД", lambda: right_motor.set_speed(40), "Должно вращаться ВПЕРЁД"),
        ("ПРАВОЕ колесо НАЗАД", lambda: right_motor.set_speed(-40), "Должно вращаться НАЗАД"),
    ]
    
    for name, action, expected in tests:
        print(f"\n▶ {name}")
        print(f"Ожидаю: {expected}")
        action()
        input("Наблюдайте и нажмите Enter чтобы продолжить...")
        left_motor.stop()
        right_motor.stop()
        time.sleep(1)
    
    print("\n" + "=" * 60)
    print("📝 РЕЗУЛЬТАТЫ ТЕСТА:")
    print("=" * 60)
    print("1. ЛЕВОЕ колесо ВПЕРЁД - вращалось правильно? (y/n): ", end="")
    left_forward_ok = input().lower() == 'y'
    
    print("2. ЛЕВОЕ колесо НАЗАД - вращалось правильно? (y/n): ", end="")
    left_backward_ok = input().lower() == 'y'
    
    print("3. ПРАВОЕ колесо ВПЕРЁД - вращалось правильно? (y/n): ", end="")
    right_forward_ok = input().lower() == 'y'
    
    print("4. ПРАВОЕ колесо НАЗАД - вращалось правильно? (y/n): ", end="")
    right_backward_ok = input().lower() == 'y'
    
    # Определяем, нужно ли менять полярность
    if not left_forward_ok or not left_backward_ok:
        print("\n⚠️  ЛЕВЫЙ мотор вращается не правильно!")
        print("   Попробуйте поменять провода IN1 и IN2 местами")
    
    if not right_forward_ok or not right_backward_ok:
        print("⚠️  ПРАВЫЙ мотор вращается не правильно!")
        print("   Попробуйте поменять провода IN3 и IN4 местами")
    
    if left_forward_ok and left_backward_ok and right_forward_ok and right_backward_ok:
        print("\n✅ Все моторы вращаются правильно!")
        return True
    else:
        print("\n❌ Нужно исправить подключение моторов!")
        return False

# ============================================================================
# ИСПРАВЛЕННЫЕ ФУНКЦИИ ДВИЖЕНИЯ
# ============================================================================

def robot_forward(speed=30):
    """Движение ВПЕРЁД - ИСПРАВЛЕНО"""
    print(f"\n▶ ВПЕРЁД: скорость {speed}%")
    left_motor.set_speed(speed)
    right_motor.set_speed(speed)

def robot_backward(speed=30):
    """Движение НАЗАД - ИСПРАВЛЕНО"""
    print(f"\n◀ НАЗАД: скорость {speed}%")
    left_motor.set_speed(-speed)
    right_motor.set_speed(-speed)

def robot_turn_left(speed=30):
    """ПОВОРОТ ВЛЕВО - ИСПРАВЛЕНО"""
    print(f"\n↰ ПОВОРОТ ВЛЕВО: скорость {speed}%")
    # Левый медленнее, правый быстрее
    left_motor.set_speed(speed * 0.3)
    right_motor.set_speed(speed)

def robot_turn_right(speed=30):
    """ПОВОРОТ ВПРАВО - ИСПРАВЛЕНО"""
    print(f"\n↱ ПОВОРОТ ВПРАВО: скорость {speed}%")
    # Левый быстрее, правый медленнее
    left_motor.set_speed(speed)
    right_motor.set_speed(speed * 0.3)

def robot_spin_left(speed=30):
    """РАЗВОРОТ НА МЕСТЕ ВЛЕВО - ИСПРАВЛЕНО"""
    print(f"\n↶ РАЗВОРОТ ВЛЕВО: скорость {speed}%")
    # Левый назад, правый вперёд
    left_motor.set_speed(-speed)
    right_motor.set_speed(speed)

def robot_spin_right(speed=30):
    """РАЗВОРОТ НА МЕСТЕ ВПРАВО - ИСПРАВЛЕНО"""
    print(f"\n↷ РАЗВОРОТ ВПРАВО: скорость {speed}%")
    # Левый вперёд, правый назад
    left_motor.set_speed(speed)
    right_motor.set_speed(-speed)

def robot_stop():
    """СТОП"""
    print("\n⏹ СТОП")
    left_motor.stop()
    right_motor.stop()

# ============================================================================
# ПРОСТОЕ РУЧНОЕ УПРАВЛЕНИЕ (без сложного меню)
# ============================================================================

def simple_manual_control():
    """Простое ручное управление - ИСПРАВЛЕНО"""
    print("\n" + "=" * 60)
    print("🎮 ПРОСТОЕ РУЧНОЕ УПРАВЛЕНИЕ")
    print("=" * 60)
    print("W - Вперёд        S - Назад")
    print("A - Влево         D - Вправо")
    print("Q - Разворот влево E - Разворот вправо")
    print("Space - Стоп      + - Увеличить скорость")
    print("- - Уменьшить скорость     X - Выход")
    print("=" * 60)
    
    speed = 50
    
    # Настройка терминала для чтения клавиш
    import termios, tty
    
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    print(f"\nТекущая скорость: {speed}%")
    print("Нажмите любую клавишу для управления...")
    
    try:
        while True:
            ch = getch().lower()
            
            if ch == 'x':
                break
            elif ch == 'w':
                robot_forward(speed)
            elif ch == 's':
                robot_backward(speed)
            elif ch == 'a':
                robot_turn_left(speed)
            elif ch == 'd':
                robot_turn_right(speed)
            elif ch == 'q':
                robot_spin_left(speed)
            elif ch == 'e':
                robot_spin_right(speed)
            elif ch == ' ':
                robot_stop()
            elif ch == '+':
                speed = min(MAX_PWM, speed + 10)
                print(f"\n📈 Скорость: {speed}%")
            elif ch == '-':
                speed = max(10, speed - 10)  # Минимум 30% из-за мёртвой зоны
                print(f"\n📉 Скорость: {speed}%")
            else:
                print(f"\n? Неизвестная команда: {ch}")
            
            # Вывод счётчиков энкодеров
            print(f"Энкодеры: Л={encoder_left.get_count():4d} П={encoder_right.get_count():4d}")
            
    except KeyboardInterrupt:
        pass
    finally:
        robot_stop()
        print("\n✅ Управление завершено")

# ============================================================================
# ТЕСТ РАЗНЫХ СКОРОСТЕЙ
# ============================================================================

def test_speed_range():
    """Тест разных скоростей"""
    print("\n" + "=" * 60)
    print("⚡ ТЕСТ ДИАПАЗОНА СКОРОСТЕЙ")
    print("=" * 60)
    print("Поднимите робота!")
    input("Нажмите Enter для начала...")
    
    speeds = [30, 40, 45]
    
    for speed in speeds:
        print(f"\nТест скорости {speed}%")
        print("Левый мотор вперёд...")
        left_motor.set_speed(speed)
        time.sleep(2)
        left_motor.stop()
        time.sleep(1)
        
        print("Правый мотор вперёд...")
        right_motor.set_speed(speed)
        time.sleep(2)
        right_motor.stop()
        time.sleep(1)
    
    robot_stop()
    print("\n✅ Тест скоростей завершён")

# ============================================================================
# ГЛАВНАЯ ФУНКЦИЯ
# ============================================================================

def main():
    """Основная функция"""
    print("\n" + "=" * 60)
    print("🤖 ДИАГНОСТИКА И НАСТРОЙКА РОБОТА")
    print("=" * 60)
    
    # Шаг 1: Тест направлений (САМЫЙ ВАЖНЫЙ!)
    if not test_directions():
        print("\n❌ Сначала исправьте направление моторов!")
        print("Поменяйте провода IN1/IN2 или IN3/IN4 местами")
        return
    
    # Шаг 2: Тест скоростей
    test_speed_range()
    
    # Шаг 3: Ручное управление
    simple_manual_control()

# ============================================================================
# ЗАПУСК
# ============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Программа прервана")
    except Exception as e:
        print(f"\n❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Безопасное завершение
        print("\n🔌 Завершение работы...")
        left_motor.stop()
        right_motor.stop()
        encoder_left.cleanup()
        encoder_right.cleanup()
        pi.stop()
        print("✅ Все ресурсы освобождены")