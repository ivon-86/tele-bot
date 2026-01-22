#!/usr/bin/env python3
"""
Тестовый скрипт для проверки моторов и энкодеров робота
Обновлённая версия с исправленными пинами энкодеров
"""

import pigpio
import time
import threading
import sys
import termios
import tty
import select
from collections import deque

# ============================================================================
# КОНФИГУРАЦИЯ ПИНОВ (исправленная версия)
# ============================================================================

# Правый мотор
RIGHT_PWM_PIN = 13     # GPIO13 (PWM1)
RIGHT_IN1_PIN = 19     # GPIO19 (IN3 на L298N)
RIGHT_IN2_PIN = 26     # GPIO26 (IN4 на L298N)

# Левый мотор  
LEFT_PWM_PIN = 18      # GPIO18 (PWM0)
LEFT_IN1_PIN = 20      # GPIO20 (IN1 на L298N)
LEFT_IN2_PIN = 21      # GPIO21 (IN2 на L298N)

# Энкодеры (ИСПРАВЛЕННЫЕ ПИНЫ)
RIGHT_ENC_A = 5        # GPIO5 (S1 правого мотора)
RIGHT_ENC_B = 6        # GPIO6 (S2 правого мотора) - ИСПРАВЛЕНО!
LEFT_ENC_A = 17        # GPIO17 (S1 левого мотора)
LEFT_ENC_B = 27        # GPIO27 (S2 левого мотора)

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ PIGPIO
# ============================================================================

pi = pigpio.pi()
if not pi.connected:
    print("Ошибка: Не удалось подключиться к pigpio демону")
    print("Запустите: sudo pigpiod")
    sys.exit(1)

# ============================================================================
# КЛАСС ДЛЯ УПРАВЛЕНИЯ МОТОРОМ
# ============================================================================

class Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin, name="Motor"):
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.name = name
        self.speed = 0  # 0-100%
        self.target_speed = 0
        self.is_smoothing = False
        
        # Настройка пинов
        pi.set_mode(pwm_pin, pigpio.OUTPUT)
        pi.set_mode(in1_pin, pigpio.OUTPUT)
        pi.set_mode(in2_pin, pigpio.OUTPUT)
        
        # Инициализация ШИМ
        pi.set_PWM_frequency(pwm_pin, 1000)  # Частота 1кГц
        pi.set_PWM_range(pwm_pin, 100)       # Диапазон 0-100
        pi.set_PWM_dutycycle(pwm_pin, 0)     # Выключено
        
        # Установка направления
        pi.write(in1_pin, 0)
        pi.write(in2_pin, 0)
        
        print(f"{name} инициализирован на пинах: PWM={pwm_pin}, IN1={in1_pin}, IN2={in2_pin}")
    
    def set_speed(self, speed, immediate=False):
        """Установка скорости от -100 до 100"""
        # Ограничиваем скорость
        speed = max(-100, min(100, speed))
        self.target_speed = speed
        
        if immediate:
            self._apply_speed(speed)
        else:
            # Запускаем плавное изменение, если не уже в процессе
            if not self.is_smoothing:
                threading.Thread(target=self._smooth_to_target, daemon=True).start()
    
    def _apply_speed(self, speed):
        """Непосредственное применение скорости"""
        self.speed = speed
        
        # Управление направлением
        if speed > 0:
            # Вперёд
            pi.write(self.in1_pin, 1)
            pi.write(self.in2_pin, 0)
            pwm_value = speed
        elif speed < 0:
            # Назад
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 1)
            pwm_value = -speed
        else:
            # Стоп
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 0)
            pwm_value = 0
        
        # Установка ШИМ
        pi.set_PWM_dutycycle(self.pwm_pin, pwm_value)
    
    def _smooth_to_target(self, duration=0.5, steps=20):
        """Плавное изменение скорости до целевого значения"""
        if self.is_smoothing:
            return
            
        self.is_smoothing = True
        start_speed = self.speed
        step_time = duration / steps
        step_value = (self.target_speed - start_speed) / steps
        
        for i in range(steps):
            current_speed = start_speed + step_value * (i + 1)
            self._apply_speed(current_speed)
            time.sleep(step_time)
        
        # Финальная установка точного значения
        self._apply_speed(self.target_speed)
        self.is_smoothing = False
    
    def stop(self):
        """Полная остановка"""
        self.set_speed(0, immediate=True)
    
    def brake(self):
        """Торможение (короткое замыкание обмоток)"""
        pi.write(self.in1_pin, 1)
        pi.write(self.in2_pin, 1)
        pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.speed = 0
        self.target_speed = 0
        print(f"{self.name}: ТОРМОЖЕНИЕ")

# ============================================================================
# КЛАСС ДЛЯ ЧТЕНИЯ ЭНКОДЕРА С RPM И СРЕДНЕЙ СКОРОСТЬЮ
# ============================================================================

class Encoder:
    def __init__(self, pin_a, pin_b, name="Encoder", pulses_per_rev=4):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.pulses_per_rev = pulses_per_rev  # 2 импульса × 2 канала = 4 на оборот
        
        self.count = 0
        self.total_count = 0
        self.direction = 0  # 1=вперёд, -1=назад, 0=неизвестно
        self.rpm = 0.0
        self.velocity = 0.0  # мм/сек
        
        # Для расчёта RPM и скорости
        self.last_time = time.time()
        self.last_count = 0
        self.rpm_history = deque(maxlen=10)
        self.velocity_history = deque(maxlen=10)
        
        # Физические параметры (примерные, нужно уточнить)
        self.wheel_diameter_mm = 65  # Диаметр колеса в мм
        self.wheel_circumference_mm = 3.14159 * self.wheel_diameter_mm
        
        # Настройка пинов как входов
        pi.set_mode(pin_a, pigpio.INPUT)
        pi.set_mode(pin_b, pigpio.INPUT)
        pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Получаем начальные состояния
        self.last_a = pi.read(pin_a)
        self.last_b = pi.read(pin_b)
        
        # Callback на изменения
        self.cb_a = pi.callback(pin_a, pigpio.EITHER_EDGE, self._callback)
        self.cb_b = pi.callback(pin_b, pigpio.EITHER_EDGE, self._callback)
        
        # Запуск потока для расчёта RPM
        self.running = True
        self.thread = threading.Thread(target=self._update_rpm, daemon=True)
        self.thread.start()
        
        print(f"{name} инициализирован на пинах: A={pin_a}, B={pin_b}")
        print(f"  Предполагается {pulses_per_rev} импульса на оборот")
        print(f"  Диаметр колеса: {self.wheel_diameter_mm}мм")
    
    def _callback(self, gpio, level, tick):
        """Обработчик изменений на энкодере"""
        current_a = pi.read(self.pin_a)
        current_b = pi.read(self.pin_b)
        
        # Определяем изменение
        if gpio == self.pin_a and self.last_a != current_a:
            # Изменился канал A
            if current_a == 1:  # Передний фронт A
                if current_b == 0:
                    self.count += 1
                    self.total_count += 1
                    self.direction = 1
                else:
                    self.count -= 1
                    self.total_count -= 1
                    self.direction = -1
            else:  # Задний фронт A
                if current_b == 1:
                    self.count += 1
                    self.total_count += 1
                    self.direction = 1
                else:
                    self.count -= 1
                    self.total_count -= 1
                    self.direction = -1
            self.last_a = current_a
            
        elif gpio == self.pin_b and self.last_b != current_b:
            # Изменился канал B
            if current_b == 1:  # Передний фронт B
                if current_a == 1:
                    self.count += 1
                    self.total_count += 1
                    self.direction = 1
                else:
                    self.count -= 1
                    self.total_count -= 1
                    self.direction = -1
            else:  # Задний фронт B
                if current_a == 0:
                    self.count += 1
                    self.total_count += 1
                    self.direction = 1
                else:
                    self.count -= 1
                    self.total_count -= 1
                    self.direction = -1
            self.last_b = current_b
    
    def _update_rpm(self):
        """Фоновая задача для расчёта RPM и скорости"""
        while self.running:
            time.sleep(0.1)  # Обновляем каждые 100мс
            
            current_time = time.time()
            time_diff = current_time - self.last_time
            
            if time_diff > 0:
                count_diff = self.count - self.last_count
                
                # Расчёт RPM
                revolutions = count_diff / self.pulses_per_rev
                self.rpm = (revolutions / time_diff) * 60
                self.rpm_history.append(self.rpm)
                
                # Расчёт скорости (мм/сек)
                distance_mm = (count_diff / self.pulses_per_rev) * self.wheel_circumference_mm
                self.velocity = distance_mm / time_diff
                self.velocity_history.append(self.velocity)
                
                # Обновляем для следующего измерения
                self.last_count = self.count
                self.last_time = current_time
    
    def get_rpm(self):
        """Получить текущее RPM"""
        return self.rpm
    
    def get_avg_rpm(self):
        """Получить среднее RPM"""
        if self.rpm_history:
            return sum(self.rpm_history) / len(self.rpm_history)
        return 0.0
    
    def get_velocity(self):
        """Получить текущую скорость (мм/сек)"""
        return self.velocity
    
    def get_avg_velocity(self):
        """Получить среднюю скорость"""
        if self.velocity_history:
            return sum(self.velocity_history) / len(self.velocity_history)
        return 0.0
    
    def get_distance(self):
        """Получить пройденное расстояние (мм)"""
        return (self.total_count / self.pulses_per_rev) * self.wheel_circumference_mm
    
    def get_count(self):
        """Получить текущее количество импульсов (с последнего сброса)"""
        return self.count
    
    def get_total_count(self):
        """Получить общее количество импульсов"""
        return self.total_count
    
    def get_direction(self):
        """Получить направление движения"""
        return self.direction
    
    def reset(self):
        """Сброс счётчика (но не общего!)"""
        self.count = 0
        self.direction = 0
        self.last_count = 0
        self.last_time = time.time()
    
    def reset_total(self):
        """Полный сброс (включая общий счётчик)"""
        self.count = 0
        self.total_count = 0
        self.direction = 0
        self.last_count = 0
        self.last_time = time.time()
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1)
        if hasattr(self, 'cb_a'):
            self.cb_a.cancel()
        if hasattr(self, 'cb_b'):
            self.cb_b.cancel()

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ
# ============================================================================

print("=" * 60)
print("🤖 ТЕСТОВАЯ СИСТЕМА РОБОТА С ЭНКОДЕРАМИ")
print("=" * 60)

# Создаём моторы
left_motor = Motor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, "Левый мотор")
right_motor = Motor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, "Правый мотор")

# Создаём энкодеры (теперь без конфликтов!)
encoder_left = Encoder(LEFT_ENC_A, LEFT_ENC_B, "Энкодер левый", pulses_per_rev=4)
encoder_right = Encoder(RIGHT_ENC_A, RIGHT_ENC_B, "Энкодер правый", pulses_per_rev=4)

print(f"\n✅ Конфигурация пинов:")
print(f"   Левый мотор:  PWM={LEFT_PWM_PIN}, IN1={LEFT_IN1_PIN}, IN2={LEFT_IN2_PIN}")
print(f"   Правый мотор: PWM={RIGHT_PWM_PIN}, IN1={RIGHT_IN1_PIN}, IN2={RIGHT_IN2_PIN}")
print(f"   Левый энкодер: A={LEFT_ENC_A}, B={LEFT_ENC_B}")
print(f"   Правый энкодер: A={RIGHT_ENC_A}, B={RIGHT_ENC_B}")
print("\n🚀 Система готова к тестированию!")
time.sleep(1)

# ============================================================================
# ФУНКЦИИ ДЛЯ ТЕСТИРОВАНИЯ
# ============================================================================

def test_basic_movements():
    """Тест базовых движений с выводом данных энкодеров"""
    print("\n" + "=" * 60)
    print("🔄 ТЕСТ БАЗОВЫХ ДВИЖЕНИЙ")
    print("=" * 60)
    
    movements = [
        ("Вперёд 3 сек", lambda: (left_motor.set_speed(50), right_motor.set_speed(50)), 3),
        ("Стоп", lambda: (left_motor.stop(), right_motor.stop()), 1),
        ("Назад 3 сек", lambda: (left_motor.set_speed(-50), right_motor.set_speed(-50)), 3),
        ("Стоп", lambda: (left_motor.stop(), right_motor.stop()), 1),
        ("Поворот влево 2 сек", lambda: (left_motor.set_speed(30), right_motor.set_speed(50)), 2),
        ("Стоп", lambda: (left_motor.stop(), right_motor.stop()), 1),
        ("Поворот вправо 2 сек", lambda: (left_motor.set_speed(50), right_motor.set_speed(30)), 2),
        ("Стоп", lambda: (left_motor.stop(), right_motor.stop()), 1),
        ("Разворот влево 2 сек", lambda: (left_motor.set_speed(-50), right_motor.set_speed(50)), 2),
        ("Стоп", lambda: (left_motor.stop(), right_motor.stop()), 1),
        ("Разворот вправо 2 сек", lambda: (left_motor.set_speed(50), right_motor.set_speed(-50)), 2),
        ("Стоп", lambda: (left_motor.stop(), right_motor.stop()), 1),
    ]
    
    for name, action, duration in movements:
        print(f"\n▶ {name}")
        action()
        
        # Выводим данные энкодеров во время движения
        start_time = time.time()
        while time.time() - start_time < duration:
            print_encoder_status(brief=True)
            time.sleep(0.2)
        
        print_encoder_status()

def test_encoder_calibration():
    """Калибровка энкодеров - измерение реальных импульсов на оборот"""
    print("\n" + "=" * 60)
    print("🎯 КАЛИБРОВКА ЭНКОДЕРОВ")
    print("=" * 60)
    print("1. Поднимите робота, чтобы колёса свободно вращались")
    print("2. Будет проверено, сколько импульсов на полный оборот")
    print("3. Нажмите Enter для начала калибровки...")
    input()
    
    # Сбрасываем счётчики
    encoder_left.reset_total()
    encoder_right.reset_total()
    
    print("\nКалибровка левого мотора:")
    print("Вращайте левое колесо вручную на ОДИН полный оборот")
    print("Нажмите Enter после завершения оборота...")
    input()
    left_pulses = encoder_left.get_total_count()
    print(f"Левый энкодер: {left_pulses} импульсов за оборот")
    
    encoder_left.reset_total()
    
    print("\nКалибровка правого мотора:")
    print("Вращайте правое колесо вручную на ОДИН полный оборот")
    print("Нажмите Enter после завершения оборота...")
    input()
    right_pulses = encoder_right.get_total_count()
    print(f"Правый энкодер: {right_pulses} импульсов за оборот")
    
    print(f"\n📊 РЕЗУЛЬТАТ КАЛИБРОВКИ:")
    print(f"   Левый: {left_pulses} имп/оборот (ожидалось 4)")
    print(f"   Правый: {right_pulses} имп/оборот (ожидалось 4)")
    
    if abs(left_pulses - 4) > 1 or abs(right_pulses - 4) > 1:
        print("\n⚠️  ВНИМАНИЕ: Количество импульсов отличается от ожидаемого!")
        print("   Возможно, нужно обновить параметр 'pulses_per_rev'")
    
    encoder_left.reset_total()
    encoder_right.reset_total()

def test_speed_synchronization():
    """Тест синхронизации скорости моторов по энкодерам"""
    print("\n" + "=" * 60)
    print("⚡ ТЕСТ СИНХРОНИЗАЦИИ СКОРОСТИ")
    print("=" * 60)
    print("Задача: двигаться прямо, корректируя скорость по энкодерам")
    
    # ПИД-коэффициенты (упрощённые)
    Kp = 0.5  # Пропорциональный коэффициент
    
    target_rpm = 100  # Целевое RPM
    
    print(f"\nЦелевая скорость: {target_rpm} RPM")
    print("Запуск через 3 секунды...")
    time.sleep(3)
    
    # Сбрасываем энкодеры
    encoder_left.reset()
    encoder_right.reset()
    
    # Устанавливаем начальную скорость
    base_speed = 50
    left_motor.set_speed(base_speed)
    right_motor.set_speed(base_speed)
    
    # Цикл коррекции
    print("\nЛевый RPM | Правый RPM | Коррекция")
    print("-" * 40)
    
    for i in range(20):  # 20 итераций по 0.5 сек = 10 секунд
        left_rpm = encoder_left.get_avg_rpm()
        right_rpm = encoder_right.get_avg_rpm()
        
        # Простейшая коррекция
        error = left_rpm - right_rpm
        correction = error * Kp
        
        # Применяем коррекцию
        new_left_speed = base_speed - correction
        new_right_speed = base_speed + correction
        
        # Ограничиваем скорости
        new_left_speed = max(0, min(100, new_left_speed))
        new_right_speed = max(0, min(100, new_right_speed))
        
        left_motor.set_speed(new_left_speed, immediate=True)
        right_motor.set_speed(new_right_speed, immediate=True)
        
        print(f"{left_rpm:7.1f} | {right_rpm:7.1f} | {correction:+.2f}")
        time.sleep(0.5)
    
    print("\nЗавершение теста...")
    left_motor.stop()
    right_motor.stop()

def monitor_encoders_real_time(duration=10):
    """Мониторинг энкодеров в реальном времени"""
    print(f"\n📊 МОНИТОРИНГ ЭНКОДЕРОВ ({duration} сек)")
    print("=" * 60)
    print("Время | Левый RPM | Правый RPM | Левый V | Правый V | Разница RPM")
    print("-" * 80)
    
    start_time = time.time()
    encoder_left.reset()
    encoder_right.reset()
    
    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        left_rpm = encoder_left.get_rpm()
        right_rpm = encoder_right.get_rpm()
        left_v = encoder_left.get_velocity()
        right_v = encoder_right.get_velocity()
        diff = left_rpm - right_rpm
        
        print(f"{elapsed:5.1f} | {left_rpm:7.1f} | {right_rpm:7.1f} | "
              f"{left_v:6.1f} | {right_v:6.1f} | {diff:+.1f}")
        
        time.sleep(0.2)
    
    # Итоговая статистика
    print("\n📈 ИТОГОВАЯ СТАТИСТИКА:")
    print(f"  Левый мотор:  {encoder_left.get_count()} имп, {encoder_left.get_distance():.0f} мм")
    print(f"  Правый мотор: {encoder_right.get_count()} имп, {encoder_right.get_distance():.0f} мм")
    print(f"  Разница: {abs(encoder_left.get_count() - encoder_right.get_count())} имп")

def print_encoder_status(brief=False):
    """Вывод статуса энкодеров"""
    if brief:
        print(f"Л: {encoder_left.get_count():4d} имп ({encoder_left.get_rpm():5.1f} RPM) | "
              f"П: {encoder_right.get_count():4d} имп ({encoder_right.get_rpm():5.1f} RPM)")
    else:
        print(f"\n📊 СТАТУС ЭНКОДЕРОВ:")
        print(f"  Левый:  {encoder_left.get_count():6d} имп | {encoder_left.get_rpm():6.1f} RPM | "
              f"{encoder_left.get_velocity():6.1f} мм/сек | {encoder_left.get_distance():6.0f} мм")
        print(f"  Правый: {encoder_right.get_count():6d} имп | {encoder_right.get_rpm():6.1f} RPM | "
              f"{encoder_right.get_velocity():6.1f} мм/сек | {encoder_right.get_distance():6.0f} мм")

# ============================================================================
# ГЛАВНОЕ МЕНЮ
# ============================================================================

def main_menu():
    """Главное меню тестирования"""
    while True:
        print("\n" + "=" * 60)
        print("🤖 ГЛАВНОЕ МЕНЮ ТЕСТИРОВАНИЯ РОБОТА")
        print("=" * 60)
        print("1. Тест базовых движений (с мониторингом энкодеров)")
        print("2. Калибровка энкодеров")
        print("3. Тест синхронизации скорости")
        print("4. Мониторинг энкодеров в реальном времени")
        print("5. Ручное управление с клавиатуры")
        print("6. Вывод текущего статуса энкодеров")
        print("7. Сброс счётчиков энкодеров")
        print("8. Тест плавного движения")
        print("0. Выход")
        print("=" * 60)
        
        choice = input("Выберите опцию (0-8): ").strip()
        
        if choice == '0':
            print("\n🚪 Выход из программы...")
            break
            
        elif choice == '1':
            test_basic_movements()
            
        elif choice == '2':
            test_encoder_calibration()
            
        elif choice == '3':
            test_speed_synchronization()
            
        elif choice == '4':
            try:
                duration = float(input("Длительность мониторинга (сек): ") or "10")
                monitor_encoders_real_time(duration)
            except ValueError:
                print("❌ Ошибка: введите число")
                
        elif choice == '5':
            manual_control()
            
        elif choice == '6':
            print_encoder_status()
            
        elif choice == '7':
            encoder_left.reset()
            encoder_right.reset()
            print("✅ Счётчики энкодеров сброшены")
            
        elif choice == '8':
            test_smooth_movement()
            
        else:
            print("❌ Неверный выбор. Попробуйте снова.")
        
        input("\nНажмите Enter для продолжения...")

def manual_control():
    """Ручное управление с клавиатуры (упрощённая версия)"""
    print("\n" + "=" * 60)
    print("🎮 РУЧНОЕ УПРАВЛЕНИЕ С КЛАВИАТУРЫ")
    print("=" * 60)
    print("Управление: W-вперёд, S-назад, A-влево, D-вправо")
    print("          Q-разворот влево, E-разворот вправо")
    print("          Space-стоп, B-торможение, ESC-выход")
    print("=" * 60)
    
    speed = 50
    
    import termios, tty, select
    
    def get_key():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                return key
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    try:
        while True:
            key = get_key().lower()
            
            if key == '\x1b':  # ESC
                break
            elif key == 'w':
                left_motor.set_speed(speed)
                right_motor.set_speed(speed)
                print(f"▶ Вперёд {speed}%")
            elif key == 's':
                left_motor.set_speed(-speed)
                right_motor.set_speed(-speed)
                print(f"◀ Назад {speed}%")
            elif key == 'a':
                left_motor.set_speed(speed * 0.3)
                right_motor.set_speed(speed)
                print(f"↰ Влево")
            elif key == 'd':
                left_motor.set_speed(speed)
                right_motor.set_speed(speed * 0.3)
                print(f"↱ Вправо")
            elif key == 'q':
                left_motor.set_speed(-speed)
                right_motor.set_speed(speed)
                print(f"↶ Разворот влево")
            elif key == 'e':
                left_motor.set_speed(speed)
                right_motor.set_speed(-speed)
                print(f"↷ Разворот вправо")
            elif key == ' ':
                left_motor.stop()
                right_motor.stop()
                print("⏹ Стоп")
            elif key == 'b':
                left_motor.brake()
                right_motor.brake()
                print("⚠ Торможение")
            elif key == '+':
                speed = min(100, speed + 10)
                print(f"📈 Скорость: {speed}%")
            elif key == '-':
                speed = max(10, speed - 10)
                print(f"📉 Скорость: {speed}%")
            
            # Выводим статус каждые 5 итераций
            if random.random() < 0.2:  # Примерно каждые 5 нажатий
                print_encoder_status(brief=True)
                
    except KeyboardInterrupt:
        pass
    finally:
        left_motor.stop()
        right_motor.stop()
        print("\n✅ Ручное управление завершено")

def test_smooth_movement():
    """Тест плавного движения"""
    print("\n" + "=" * 60)
    print("🌊 ТЕСТ ПЛАВНОГО ДВИЖЕНИЯ")
    print("=" * 60)
    
    print("1. Плавный разгон от 0 до 60% за 3 секунды")
    left_motor.set_speed(60)
    right_motor.set_speed(60)
    time.sleep(2)
    
    print("2. Движение вперёд 3 секунды")
    monitor_encoders_real_time(3)
    
    print("3. Плавное торможение до 0 за 2 секунды")
    left_motor.set_speed(0)
    right_motor.set_speed(0)
    time.sleep(2)
    
    print("4. Плавное движение назад")
    left_motor.set_speed(-40)
    right_motor.set_speed(-40)
    time.sleep(2)
    
    print("5. Плавная остановка")
    left_motor.set_speed(0)
    right_motor.set_speed(0)
    
    print("\n✅ Тест плавного движения завершён")

# ============================================================================
# ЗАПУСК ПРОГРАММЫ
# ============================================================================

if __name__ == "__main__":
    import random
    
    print("\n" + "=" * 60)
    print("🤖 СИСТЕМА ТЕСТИРОВАНИЯ РОБОТА С ЭНКОДЕРАМИ")
    print("=" * 60)
    print(f"Дата: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Конфигурация подтверждена - конфликтов пинов нет")
    print("\nЗапуск главного меню через 2 секунды...")
    time.sleep(2)
    
    try:
        main_menu()
    except KeyboardInterrupt:
        print("\n\n🛑 Программа прервана пользователем")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Аварийная остановка
        print("\n🔌 Завершение работы и очистка ресурсов...")
        left_motor.stop()
        right_motor.stop()
        encoder_left.cleanup()
        encoder_right.cleanup()
        pi.stop()
        print("✅ Все ресурсы освобождены. До свидания!")