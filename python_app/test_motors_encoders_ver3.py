#!/usr/bin/env python3
"""
УПРОЩЁННЫЙ И ИСПРАВЛЕННЫЙ скрипт управления роботом
Без сложной многопоточности, с работающей синхронизацией
"""

import pigpio
import time
import sys
import termios
import tty
import select
import math

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
PWM_FREQUENCY = 450    # Частота ШИМ в Гц
MAX_PWM = 100           # Максимальный ШИМ в % (ограничиваем ток)
MIN_PWM = 20           # Минимальный рабочий ШИМ
START_PWM = 25         # Стартовый ШИМ (для плавного старта)

# Настройки плавного старта
SMOOTH_START_TIME = 0.25  # Время плавного старта в секундах

# Настройки синхронизации
SYNC_ENABLED = True    # Включить синхронизацию скорости
SYNC_CORRECTION = 0.5  # Коэффициент коррекции (0.0-1.0)
SYNC_UPDATE_TIME = 0.1 # Время между коррекциями

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ PIGPIO
# ============================================================================

pi = pigpio.pi()
if not pi.connected:
    print("Ошибка: Не удалось подключиться к pigpio демону")
    print("Запустите: sudo pigpiod")
    sys.exit(1)

# ============================================================================
# ПРОСТОЙ И НАДЁЖНЫЙ КЛАСС ЭНКОДЕРА
# ============================================================================

class SimpleEncoder:
    def __init__(self, pin_a, pin_b, name="Encoder"):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        
        # Счётчики
        self.count = 0
        self.total_pulses = 0
        
        # Для расчёта RPM
        self.last_count = 0
        self.last_time = time.time()
        self.rpm = 0.0
        
        # Настройка пинов
        pi.set_mode(pin_a, pigpio.INPUT)
        pi.set_mode(pin_b, pigpio.INPUT)
        pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Только один callback на канал A для простоты
        self.cb = pi.callback(pin_a, pigpio.EITHER_EDGE, self._pulse_callback)
        
        print(f"{name}: A={pin_a}, B={pin_b}")
    
    def _pulse_callback(self, gpio, level, tick):
        """Обработчик импульса - просто считаем"""
        self.count += 1
        self.total_pulses += 1
    
    def update_rpm(self):
        """Обновить расчёт RPM"""
        current_time = time.time()
        time_diff = current_time - self.last_time
        
        if time_diff > 0.1:  # Обновляем каждые 100мс
            pulses = self.count - self.last_count
            
            # 4 импульса на оборот (2 датчика Холла × 2 фронта)
            revolutions = pulses / 4.0
            self.rpm = (revolutions / time_diff) * 60.0
            
            # Для отладки: ограничиваем разумные значения
            if self.rpm > 2000:  # Нереальные значения - что-то не так
                self.rpm = 0
            
            self.last_count = self.count
            self.last_time = current_time
    
    def get_rpm(self):
        """Получить текущее RPM"""
        self.update_rpm()
        return self.rpm
    
    def get_count(self):
        return self.count
    
    def reset(self):
        self.count = 0
        self.last_count = 0
        self.last_time = time.time()
        self.rpm = 0.0
    
    def cleanup(self):
        if hasattr(self, 'cb'):
            self.cb.cancel()

# ============================================================================
# УПРОЩЁННЫЙ КЛАСС МОТОРА С ПЛАВНЫМ СТАРТОМ
# ============================================================================

class SimpleMotor:
    def __init__(self, pwm_pin, in1_pin, in2_pin, name="Motor"):
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.name = name
        
        # Состояние
        self.current_speed = 0
        self.target_speed = 0
        self.last_speed_change = 0
        
        # Настройка пинов
        pi.set_mode(pwm_pin, pigpio.OUTPUT)
        pi.set_mode(in1_pin, pigpio.OUTPUT)
        pi.set_mode(in2_pin, pigpio.OUTPUT)
        
        # Инициализация ШИМ
        pi.set_PWM_frequency(pwm_pin, PWM_FREQUENCY)
        pi.set_PWM_range(pwm_pin, 100)
        pi.set_PWM_dutycycle(pwm_pin, 0)
        
        # Установка направления
        pi.write(in1_pin, 0)
        pi.write(in2_pin, 0)
        
        print(f"{name}: PWM={pwm_pin}, IN1={in1_pin}, IN2={in2_pin}")
    
    def _apply_speed(self, speed):
        """Непосредственное применение скорости"""
        # Ограничиваем скорость
        speed = max(-MAX_PWM, min(MAX_PWM, speed))
        
        # Управление направлением
        if speed > 0:
            # ВПЕРЁД
            pi.write(self.in1_pin, 1)
            pi.write(self.in2_pin, 0)
            # Применяем минимальный ШИМ для L298N
            pwm_value = max(MIN_PWM, speed)
        elif speed < 0:
            # НАЗАД
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 1)
            pwm_value = max(MIN_PWM, -speed)
        else:
            # СТОП
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 0)
            pwm_value = 0
        
        # Установка ШИМ
        pi.set_PWM_dutycycle(self.pwm_pin, pwm_value)
        
        self.current_speed = speed
        return speed
    
    def set_speed_smooth(self, target_speed):
        """Плавная установка скорости"""
        self.target_speed = target_speed
        
        # Если уже на целевой скорости - ничего не делаем
        if abs(self.current_speed - target_speed) < 1:
            return
        
        # Определяем направление изменения
        if target_speed > self.current_speed:
            step = 1
        else:
            step = -1
        
        # Плавный разгон/торможение
        steps = int(abs(target_speed - self.current_speed))
        
        for i in range(steps):
            new_speed = self.current_speed + step
            self._apply_speed(new_speed)
            time.sleep(SMOOTH_START_TIME / max(steps, 1))
        
        # Финальная установка точного значения
        self._apply_speed(target_speed)
    
    def set_speed(self, speed, smooth=True):
        """Установка скорости"""
        if smooth and time.time() - self.last_speed_change > 0.1:
            self.set_speed_smooth(speed)
        else:
            self._apply_speed(speed)
        
        self.last_speed_change = time.time()
    
    def stop(self):
        """Остановка"""
        self._apply_speed(0)
    
    def brake(self):
        """Торможение"""
        pi.write(self.in1_pin, 1)
        pi.write(self.in2_pin, 1)
        pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.current_speed = 0
        print(f"{self.name}: ТОРМОЖЕНИЕ")

# ============================================================================
# ПРОСТОЙ КОНТРОЛЛЕР РОБОТА БЕЗ МНОГОПОТОЧНОСТИ
# ============================================================================

class SimpleRobot:
    def __init__(self):
        # Создаём моторы
        self.left_motor = SimpleMotor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, "Левый")
        self.right_motor = SimpleMotor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, "Правый")
        
        # Создаём энкодеры
        self.left_encoder = SimpleEncoder(LEFT_ENC_A, LEFT_ENC_B, "Левый энк.")
        self.right_encoder = SimpleEncoder(RIGHT_ENC_A, RIGHT_ENC_B, "Правый энк.")
        
        # Состояние
        self.sync_enabled = SYNC_ENABLED
        self.last_sync_time = 0
        self.left_counts_history = []
        self.right_counts_history = []
        
        print("\n" + "=" * 60)
        print("🤖 ПРОСТОЙ КОНТРОЛЛЕР РОБОТА")
        print("=" * 60)
        print(f"MAX_PWM: {MAX_PWM}%, MIN_PWM: {MIN_PWM}%")
        print(f"Плавный старт: {SMOOTH_START_TIME} сек")
        print(f"Синхронизация: {'ВКЛ' if SYNC_ENABLED else 'ВЫКЛ'}")
        print("=" * 60)
    
    def update_sync(self):
        """Простая синхронизация скорости"""
        if not self.sync_enabled:
            return
        
        current_time = time.time()
        if current_time - self.last_sync_time < SYNC_UPDATE_TIME:
            return
        
        # Получаем текущие показания
        left_count = self.left_encoder.get_count()
        right_count = self.right_encoder.get_count()
        
        # Сохраняем в историю (последние 5 измерений)
        self.left_counts_history.append(left_count)
        self.right_counts_history.append(right_count)
        
        if len(self.left_counts_history) > 5:
            self.left_counts_history.pop(0)
            self.right_counts_history.pop(0)
        
        # Если история накопилась
        if len(self.left_counts_history) >= 3:
            # Вычисляем разницу скоростей
            left_diff = self.left_counts_history[-1] - self.left_counts_history[0]
            right_diff = self.right_counts_history[-1] - self.right_counts_history[0]
            
            # Если оба мотора работают
            if abs(self.left_motor.current_speed) > 10 and abs(self.right_motor.current_speed) > 10:
                # Вычисляем разницу
                diff = left_diff - right_diff
                
                # Применяем простую коррекцию
                if abs(diff) > 2:  # Порог чувствительности
                    correction = diff * SYNC_CORRECTION
                    
                    # Ограничиваем коррекцию
                    correction = max(-10, min(10, correction))
                    
                    # Применяем к моторам
                    new_left = self.left_motor.current_speed - correction
                    new_right = self.right_motor.current_speed + correction
                    
                    # Ограничиваем
                    new_left = max(-MAX_PWM, min(MAX_PWM, new_left))
                    new_right = max(-MAX_PWM, min(MAX_PWM, new_right))
                    
                    # Применяем без плавности (иначе будут задержки)
                    self.left_motor._apply_speed(new_left)
                    self.right_motor._apply_speed(new_right)
        
        self.last_sync_time = current_time
    
    def move(self, left_speed, right_speed):
        """Движение робота"""
        self.left_motor.set_speed(left_speed)
        self.right_motor.set_speed(right_speed)
        
        # Сбрасываем историю при смене направления
        if (left_speed * self.left_motor.current_speed < 0 or
            right_speed * self.right_motor.current_speed < 0):
            self.left_counts_history = []
            self.right_counts_history = []
    
    def forward(self, speed):
        self.move(speed, speed)
        print(f"▶ ВПЕРЁД {speed}%")
    
    def backward(self, speed):
        self.move(-speed, -speed)
        print(f"◀ НАЗАД {speed}%")
    
    def turn_left(self, speed):
        self.move(speed * 0.4, speed)
        print(f"↰ ВЛЕВО {speed}%")
    
    def turn_right(self, speed):
        self.move(speed, speed * 0.4)
        print(f"↱ ВПРАВО {speed}%")
    
    def spin_left(self, speed):
        self.move(-speed * 0.7, speed * 0.7)
        print(f"↶ РАЗВОРОТ ВЛЕВО {speed}%")
    
    def spin_right(self, speed):
        self.move(speed * 0.7, -speed * 0.7)
        print(f"↷ РАЗВОРОТ ВПРАВО {speed}%")
    
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        print("⏹ СТОП")
    
    def brake(self):
        self.left_motor.brake()
        self.right_motor.brake()
        print("⚠ ТОРМОЖЕНИЕ")
    
    def get_status(self):
        """Получить статус"""
        left_rpm = self.left_encoder.get_rpm()
        right_rpm = self.right_encoder.get_rpm()
        
        # Ограничиваем разумные значения RPM
        left_rpm = min(2000, left_rpm)
        right_rpm = min(2000, right_rpm)
        
        return {
            'left_speed': self.left_motor.current_speed,
            'right_speed': self.right_motor.current_speed,
            'left_rpm': left_rpm,
            'right_rpm': right_rpm,
            'left_count': self.left_encoder.get_count(),
            'right_count': self.right_encoder.get_count(),
        }
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.stop()
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()

# ============================================================================
# ПРОСТОЕ РУЧНОЕ УПРАВЛЕНИЕ
# ============================================================================

def simple_control():
    """Простое ручное управление"""
    robot = SimpleRobot()
    
    print("\n" + "=" * 60)
    print("🎮 ПРОСТОЕ РУЧНОЕ УПРАВЛЕНИЕ")
    print("=" * 60)
    print("Управление:")
    print("  W - Вперёд          S - Назад")
    print("  A - Влево           D - Вправо")
    print("  Q - Разворот влево  E - Разворот вправо")
    print("  Space - Стоп        B - Торможение")
    print("  + - Увеличить скорость")
    print("  - - Уменьшить скорость")
    print("  M - Вкл/Выкл синхронизацию")
    print("  X - Выход")
    print("=" * 60)
    
    speed = 40
    last_status_time = time.time()
    
    def get_key():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                ch = sys.stdin.read(1)
                return ch
            return ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    print(f"\nТекущая скорость: {speed}%")
    print("Синхронизация: " + ("ВКЛ" if robot.sync_enabled else "ВЫКЛ"))
    print("\nНажмите клавишу для управления...")
    
    try:
        while True:
            # Обновляем синхронизацию
            robot.update_sync()
            
            # Читаем клавишу
            ch = get_key()
            
            if ch:
                ch = ch.lower()
                
                if ch == 'x':
                    break
                elif ch == 'w':
                    robot.forward(speed)
                elif ch == 's':
                    robot.backward(speed)
                elif ch == 'a':
                    robot.turn_left(speed)
                elif ch == 'd':
                    robot.turn_right(speed)
                elif ch == 'q':
                    robot.spin_left(speed)
                elif ch == 'e':
                    robot.spin_right(speed)
                elif ch == ' ':
                    robot.stop()
                elif ch == 'b':
                    robot.brake()
                elif ch == '+':
                    speed = min(MAX_PWM, speed + 5)
                    print(f"\n📈 Скорость: {speed}%")
                elif ch == '-':
                    speed = max(START_PWM, speed - 5)
                    print(f"\n📉 Скорость: {speed}%")
                elif ch == 'm':
                    robot.sync_enabled = not robot.sync_enabled
                    status = "ВКЛ" if robot.sync_enabled else "ВЫКЛ"
                    print(f"\n🔄 Синхронизация: {status}")
                else:
                    print(f"\n? Неизвестная команда: {ch}")
            
            # Выводим статус каждые 0.3 секунды
            current_time = time.time()
            if current_time - last_status_time > 0.3:
                status = robot.get_status()
                
                # Очищаем строку и выводим новый статус
                sys.stdout.write('\r' + ' ' * 80 + '\r')
                sys.stdout.write(
                    f"Л:{status['left_speed']:3}%({status['left_rpm']:5.0f}RPM) "
                    f"П:{status['right_speed']:3}%({status['right_rpm']:5.0f}RPM) "
                    f"Спиды: Л={status['left_count']:5d} П={status['right_count']:5d}"
                )
                sys.stdout.flush()
                
                last_status_time = current_time
    
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("\n\n✅ Управление завершено")
        robot.cleanup()

# ============================================================================
# ТЕСТ ПЛАВНОГО СТАРТА
# ============================================================================

def test_smooth_start():
    """Тест плавного старта"""
    print("\n" + "=" * 60)
    print("🌊 ТЕСТ ПЛАВНОГО СТАРТА")
    print("=" * 60)
    
    robot = SimpleRobot()
    robot.sync_enabled = False  # Отключаем синхронизацию для чистоты теста
    
    print("Поднимите робота!")
    input("Нажмите Enter для начала...")
    
    print("\n1. Плавный старт вперёд от 0 до 50%")
    for speed in range(0, 51, 5):
        robot.left_motor.set_speed(speed)
        robot.right_motor.set_speed(speed)
        print(f"Скорость: {speed}%")
        time.sleep(0.1)
    
    time.sleep(2)
    
    print("\n2. Плавная остановка")
    for speed in range(50, -1, -5):
        robot.left_motor.set_speed(speed)
        robot.right_motor.set_speed(speed)
        print(f"Скорость: {speed}%")
        time.sleep(0.1)
    
    time.sleep(1)
    
    print("\n3. Плавный старт назад")
    for speed in range(0, -51, -5):
        robot.left_motor.set_speed(speed)
        robot.right_motor.set_speed(speed)
        print(f"Скорость: {speed}%")
        time.sleep(0.1)
    
    time.sleep(2)
    
    print("\n4. Плавная остановка")
    for speed in range(-50, 1, 5):
        robot.left_motor.set_speed(speed)
        robot.right_motor.set_speed(speed)
        print(f"Скорость: {speed}%")
        time.sleep(0.1)
    
    robot.stop()
    print("\n✅ Тест завершён")
    robot.cleanup()

# ============================================================================
# ТЕСТ СИНХРОНИЗАЦИИ
# ============================================================================

def test_sync():
    """Тест синхронизации"""
    print("\n" + "=" * 60)
    print("⚖️ ТЕСТ СИНХРОНИЗАЦИИ")
    print("=" * 60)
    
    robot = SimpleRobot()
    
    print("1. Тест без синхронизации")
    robot.sync_enabled = False
    robot.left_encoder.reset()
    robot.right_encoder.reset()
    
    robot.forward(50)
    time.sleep(5)
    
    left_no_sync = robot.left_encoder.get_count()
    right_no_sync = robot.right_encoder.get_count()
    diff_no_sync = left_no_sync - right_no_sync
    
    robot.stop()
    time.sleep(2)
    
    print("\n2. Тест с синхронизацией")
    robot.sync_enabled = True
    robot.left_encoder.reset()
    robot.right_encoder.reset()
    
    robot.forward(50)
    time.sleep(5)
    
    left_with_sync = robot.left_encoder.get_count()
    right_with_sync = robot.right_encoder.get_count()
    diff_with_sync = left_with_sync - right_with_sync
    
    robot.stop()
    
    print("\n" + "=" * 60)
    print("📊 РЕЗУЛЬТАТЫ:")
    print("=" * 60)
    print(f"Без синхронизации:")
    print(f"  Левый: {left_no_sync} имп")
    print(f"  Правый: {right_no_sync} имп")
    print(f"  Разница: {diff_no_sync} имп")
    
    print(f"\nС синхронизацией:")
    print(f"  Левый: {left_with_sync} имп")
    print(f"  Правый: {right_with_sync} имп")
    print(f"  Разница: {diff_with_sync} имп")
    
    improvement = (abs(diff_no_sync) - abs(diff_with_sync)) / max(abs(diff_no_sync), 1) * 100
    print(f"\n📈 Улучшение: {improvement:.1f}%")
    
    robot.cleanup()

# ============================================================================
# ГЛАВНОЕ МЕНЮ
# ============================================================================

def main_menu():
    """Простое меню"""
    print("\n" + "=" * 60)
    print("🤖 ПРОСТОЕ МЕНЮ УПРАВЛЕНИЯ")
    print("=" * 60)
    
    while True:
        print("\nВыберите опцию:")
        print("1. Ручное управление (рекомендуется сначала)")
        print("2. Тест плавного старта")
        print("3. Тест синхронизации")
        print("4. Показать настройки")
        print("0. Выход")
        
        try:
            choice = input("\nВаш выбор (0-4): ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                simple_control()
            elif choice == '2':
                test_smooth_start()
            elif choice == '3':
                test_sync()
            elif choice == '4':
                print(f"\n📋 НАСТРОЙКИ:")
                print(f"  MAX_PWM: {MAX_PWM}%")
                print(f"  MIN_PWM: {MIN_PWM}%")
                print(f"  START_PWM: {START_PWM}%")
                print(f"  Частота ШИМ: {PWM_FREQUENCY} Гц")
                print(f"  Плавный старт: {SMOOTH_START_TIME} сек")
                print(f"  Синхронизация: {SYNC_CORRECTION}")
            else:
                print("❌ Неверный выбор")
        
        except KeyboardInterrupt:
            print("\n\n🛑 Выход")
            break
        except Exception as e:
            print(f"❌ Ошибка: {e}")

# ============================================================================
# ЗАПУСК
# ============================================================================

if __name__ == "__main__":
    try:
        print("\n" + "=" * 60)
        print("🚀 ЗАПУСК ПРОСТОЙ СИСТЕМЫ УПРАВЛЕНИЯ")
        print("=" * 60)
        
        main_menu()
        
    except KeyboardInterrupt:
        print("\n\n🛑 Программа прервана")
    except Exception as e:
        print(f"\n❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🔌 Завершение работы...")
        pi.stop()
        print("✅ Все ресурсы освобождены")