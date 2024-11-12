/* main.c - Application main entry point */

/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/sys/byteorder.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/hogp.h>
#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>


#include "peer.h"
#include "pwm_led.h"
#include "service.h"
#include <dm.h>


#include <zephyr/shell/shell.h>

/**
 * Switch between boot protocol and report protocol mode.
 */
#define KEY_BOOTMODE_MASK DK_BTN2_MSK
/**
 * Switch CAPSLOCK state.
 *
 * @note
 * For simplicity of the code it works only in boot mode.
 */
#define KEY_CAPSLOCK_MASK DK_BTN1_MSK
/**
 * Switch CAPSLOCK state with response
 *
 * Write CAPSLOCK with response.
 * Just for testing purposes.
 * The result should be the same like usine @ref KEY_CAPSLOCK_MASK
 */
#define KEY_CAPSLOCK_RSP_MASK DK_BTN3_MSK

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

static struct bt_conn *default_conn;
static struct bt_hogp hogp;
static struct bt_conn *auth_conn;
static uint8_t capslock_state;

static void hids_on_ready(struct k_work *work);
static K_WORK_DEFINE(hids_ready_work, hids_on_ready);

bool led1 = false;
bool led2 = false;
bool led3 = false;
bool led4 = false;
int addr12[6] = {0};

// Функция обратного вызова, которая будет вызвана, когда устройство обнаружит другое устройство по BLE и фильтры совпадения будут удовлетворены
static void scan_filter_match(
    // Информация о устройстве, которое было обнаружено
    struct bt_scan_device_info *device_info,
    // Информация о совпадении фильтров
    struct bt_scan_filter_match *filter_match,
    // Флаг, указывающий, является ли устройство подключаемым
    bool connectable)
{
    // Объявляем переменную для хранения адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Проверяем, является ли совпадение фильтров действительным
    if (!filter_match->uuid.match ||
        (filter_match->uuid.count != 1)) {
        // Если совпадение фильтров не действителен, выводим сообщение об ошибке
        printk("Invalid device connected\n");
        // Выходим из функции
        return;
    }

    // Получаем UUID устройства из совпадения фильтров
    const struct bt_uuid *uuid = filter_match->uuid.uuid[0];

    // Преобразуем адрес устройства в строковое представление
    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    // Выводим сообщение о совпадении фильтров
    printk("Filters matched on UUID 0x%04x.\nAddress: %s connectable: %s\n",
           BT_UUID_16(uuid)->val,
           addr, connectable ? "yes" : "no");
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

// Функция обратного вызова, которая будет вызвана, когда устройство начинает подключаться к другому устройству по BLE
static void scan_connecting(
    // Информация о устройстве, которое было обнаружено
    struct bt_scan_device_info *device_info,
    // Подключение, которое было установлено
    struct bt_conn *conn)
{
    // Увеличиваем ссылку на подключение
    default_conn = bt_conn_ref(conn);
}
/** .. include_startingpoint_scan_rst */
// Функция обратного вызова, которая будет вызвана, когда обнаружен сигнал прямого рекламного объявления BLE
static void scan_filter_no_match(
    // Информация о устройстве, которое было обнаружено
    struct bt_scan_device_info *device_info,
    // Флаг, указывающий, является ли устройство подключаемым
    bool connectable)
{
    // Объявляем переменные для хранения ошибки и подключения
    int err;
    struct bt_conn *conn;
    char addr[BT_ADDR_LE_STR_LEN];

    // Проверяем, является ли тип рекламного объявления прямым
    if (device_info->recv_info->adv_type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        // Преобразуем адрес устройства в строковое представление
        bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));
        // Выводим сообщение о получении прямого рекламного объявления
        printk("Direct advertising received from %s\n", addr);
        // Останавливаем сканирование
        bt_scan_stop();

        // Создаем подключение к устройству
        err = bt_conn_le_create(device_info->recv_info->addr,
                                BT_CONN_LE_CREATE_CONN,
                                device_info->conn_param, &conn);

        // Если подключение было создано успешно
        if (!err) {
            // Увеличиваем ссылку на подключение
            default_conn = bt_conn_ref(conn);
            // Уменьшаем ссылку на подключение
            bt_conn_unref(conn);
        }
    }
}
/** .. include_endpoint_scan_rst */
BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
		scan_connecting_error, scan_connecting);

// Функция обратного вызова, которая будет вызвана, когда процесс обнаружения GATT завершится успешно
static void discovery_completed_cb(
    // Указатель на структуру bt_gatt_dm, которая представляет собой менеджер обнаружения GATT
    struct bt_gatt_dm *dm,
    // Дополнительный контекст, который может быть передан функции обратного вызова
    void *context)
{
    // Объявляем переменную для хранения ошибки
    int err;

    // Выводим сообщение об успехе процесса обнаружения
    printk("The discovery procedure succeeded\n");

    // Выводим информацию о найденных сервисах и характеристиках
    bt_gatt_dm_data_print(dm);

    // Инициализируем объект клиента HIDS
    err = bt_hogp_handles_assign(dm, &hogp);
    if (err) {
        // Выводим сообщение об ошибке, если инициализация не удалась
        printk("Could not init HIDS client object, error: %d\n", err);
    }

    // Освобождаем данные обнаружения
    err = bt_gatt_dm_data_release(dm);
    if (err) {
        // Выводим сообщение об ошибке, если освобождение не удалось
        printk("Could not release the discovery data, error code: %d\n", err);
    }
}

// Функция обратного вызова, которая будет вызвана, если служба не найдена во время процесса обнаружения GATT
static void discovery_service_not_found_cb(
    // Указатель на структуру bt_conn, которая представляет собой подключение
    struct bt_conn *conn,
    // Дополнительный контекст, который может быть передан функции обратного вызова
    void *context)
{
    // Выводим сообщение об ошибке, указывающее, что служба не найдена
    printk("The service could not be found during the discovery\n");
}

// Функция обратного вызова, которая будет вызвана при ошибке во время процесса обнаружения GATT
static void discovery_error_found_cb(
    // Указатель на структуру bt_conn, которая представляет собой подключение
    struct bt_conn *conn,
    // Код ошибки, который произошел во время процесса обнаружения
    int err,
    // Дополнительный контекст, который может быть передан функции обратного вызова
    void *context)
{
    // Выводим сообщение об ошибке с кодом ошибки
    printk("The discovery procedure failed with %d\n", err);
}

static const struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void gatt_discover(struct bt_conn *conn)
{
    // Объявляем переменную для хранения ошибки
    int err;

    // Если подключение не является подключением по умолчанию, выходим из функции
    if (conn != default_conn) {
        return;
    }

    // Запускаем процесс обнаружения GATT для подключения
    err = bt_gatt_dm_start(conn, BT_UUID_HIDS, &discovery_cb, NULL);
    if (err) {
        // Выводим сообщение об ошибке при запуске обнаружения
        printk("could not start the discovery procedure, error code: %d\n", err);
    }
}

int change(int x){
    switch (x)
    {
    case 48:
        return 0x0;
        break;
    case 49:
        return 0x1;
        break;
    case 50:
        return 0x2;
        break;
    case 51:
        return 0x3;
        break;
    case 52:
        return 0x4;
        break;
    case 53:
        return 0x5;
        break;
    case 54:
        return 0x6;
        break;
    case 55:
        return 0x7;
        break;
    case 56:
        return 0x8;
        break;
    case 57:
        return 0x9;
        break;
    case 65:
        return 0xA;
        break;
    case 66:
        return 0xB;
        break;
    case 67:
        return 0xC;
        break;
    case 68:
        return 0xD;
        break;
    case 69:
        return 0xE;
        break;
    case 70:
        return 0xF;
        break;
    default:
        return 0xf;
        break;
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    // Объявляем переменные для хранения ошибки и адреса устройства
    int err;
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строку
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Если произошла ошибка при подключении
    if (conn_err) {
        // Выводим сообщение об ошибке
        printk("Failed to connect to %s (%u)\n", addr, conn_err);

        // Если подключение было установлено по умолчанию
        if (conn == default_conn) {
            // Сбрасываем подключение по умолчанию
            bt_conn_unref(default_conn);
            default_conn = NULL;

            // Начинаем активный скан
            err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
            if (err) {
                // Выводим сообщение об ошибке при запуске сканирования
                printk("Scanning failed to start (err %d)\n", err);
            }
        }

        // Выходим из функции
        return;
    }

    // Выводим сообщение о успешном подключении
    printk("Connected: %s\n", addr);

    int j = 0;
    for(int i = 0; i < 6; i++) {
        addr12[i] = ((change(addr[j])<<4)) | (change(addr[j + 1]));
        j+=3;
    }
    
    // Устанавливаем уровень безопасности подключения
    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        // Выводим сообщение об ошибке при установке безопасности
        printk("Failed to set security: %d\n", err);

        // Запускаем процесс обнаружения GATT
        gatt_discover(conn);
    }
}

// Функция обратного вызова, которая будет вызвана, когда подключение BLE будет разорвано
static void disconnected(
    // Подключение, которое было разорвано
    struct bt_conn *conn,
    // Причина разрыва подключения
    uint8_t reason)
{
    // Объявляем переменные для хранения адреса устройства и ошибки
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    // Преобразуем адреса подключения в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Если есть авторизованное подключение, освобождаем его
    if (auth_conn) {
        bt_conn_unref(auth_conn);
        auth_conn = NULL;
    }

    // Выводим сообщение об ошибке о разрыве подключения
    printk("Disconnected: %s (reason %u)\n", addr, reason);

    // Проверяем, активен ли клиент HIDS
    if (bt_hogp_assign_check(&hogp)) {
        // Если клиент HIDS активен, освобождаем его
        printk("HIDS client active - releasing");
        bt_hogp_release(&hogp);
    }

    // Проверяем, является ли разоренное подключение стандартным подключением
    if (default_conn != conn) {
        // Если разоренное подключение не является стандартным подключением, возвращаемся из функции
        return;
    }

    // Освобождаем стандартное подключение
    bt_conn_unref(default_conn);
    default_conn = NULL;

    // Запускаем активный сканирование
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        // Выводим сообщение об ошибке, если сканирование не удалось запустить
        printk("Scanning failed to start (err %d)\n", err);
    }
}

// Обратный вызов, который будет вызван, когда уровень безопасности соединения BLE изменится
static void security_changed(
    // Соединение BLE, уровень безопасности которого изменился
    struct bt_conn *conn,
    // Новый уровень безопасности соединения
    bt_security_t level,
    // Код ошибки, указывающий результат изменения безопасности
    enum bt_security_err err)
{
    // Объявляем переменную для хранения адреса устройства
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес соединения в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Проверяем, был ли изменен уровень безопасности успешно
    if (!err) {
        // Если изменение уровня безопасности было успешным, выводим сообщение
        printk("Security changed: %s level %u\n", addr, level);
    } else {
        // Если изменение уровня безопасности не было успешным, выводим сообщение об ошибке
        printk("Security failed: %s level %u err %d\n", addr, level, err);
    }

    // Выполняем обнаружение GATT-сервера на соединении
    gatt_discover(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.security_changed = security_changed
};

// Функция инициализирует сканирование BLE устройств
static void scan_init(void)
{
    // Объявляем переменную для хранения ошибок
    int err;

    // Определяем параметры инициализации сканирования
    struct bt_scan_init_param scan_init = {
        // Если найдено совпадение, соединяемся с устройством
        .connect_if_match = 1,
        // Параметры сканирования не заданы
        .scan_param = NULL,
        // Параметры соединения по умолчанию
        .conn_param = BT_LE_CONN_PARAM_DEFAULT
    };

    // Инициализируем сканирование с заданными параметрами
    bt_scan_init(&scan_init);

    // Регистрируем обратный вызов для сканирования
    bt_scan_cb_register(&scan_cb);

    // Добавляем фильтр для сканирования UUID
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_HIDS);
    if (err) {
        // Если фильтр не может быть добавлен, выводим сообщение об ошибке
        printk("Scanning filters cannot be set (err %d)\n", err);

        // Возвращаемся из функции
        return;
    }

    // Включаем фильтр для сканирования UUID
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) {
        // Если фильтр не может быть включен, выводим сообщение об ошибке
        printk("Filters cannot be turned on (err %d)\n", err);
    }
}

// Обратный вызов для уведомлений о данных от устройства HOGP
static uint8_t hogp_notify_cb(
    // Устройство HOGP, которое отправляет уведомление
    struct bt_hogp *hogp,
    // Информация о репорте, который был получен
    struct bt_hogp_rep_info *rep,
    // Код ошибки, если он имеется
    uint8_t err,
    // Данные, которые были получены
    const uint8_t *data)
{
    // Если данные не были получены, останавливаем обработку
    if (!data) {
        return BT_GATT_ITER_STOP;//завершаем обработку полученных данных
    }

    if (data[0] == 0xfb && data[1] == 0x0f && data[2] == 0x00) {
        printk("button 1\n");
        led1 = !led1;
        dk_set_led(DK_LED1, led1);
    }else if (data[0] == 0x00 && data[1] == 0xb0 && data[2] == 0xff) {
        printk("button 2\n");
        led2 = !led2;
        dk_set_led(DK_LED2, led2);
    }else if (data[0] == 0x05 && data[1] == 0x00 && data[2] == 0x00) {
        printk("button 3\n");
        led3 = !led3;
        dk_set_led(DK_LED3, led3);
    }else if (data[0] == 0x00 && data[1] == 0x50 && data[2] == 0x00) {
        printk("button 4\n");
        led4 = !led4;
        dk_set_led(DK_LED4, led4);
    }else if (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00) {
        led1 = 0;
        led2 = 0;
        led3 = 0;
        led4 = 0;
        dk_set_led(DK_LED1, 0);
        dk_set_led(DK_LED2, 0);
        dk_set_led(DK_LED3, 0);
        dk_set_led(DK_LED4, 0);
    }else if (data[0] == 0x11 && data[1] == 0x10 && data[2] == 0x01) {
        printk("Starting Distence Measurement\n");
        struct dm_request req;

        //Запись адреса устройства
        for(int i = 0; i < 6; i++){
            req.bt_addr.a.val[5-i] = addr12[i];
        }

        req.role = DM_ROLE_INITIATOR;
        req.ranging_mode = DM_RANGING_MODE_MCPD; //peer_ranging_mode_get();
	    req.rng_seed = 1549329102;//sys_le32_to_cpu(recv_mfg_data->rng_seed) + scanner_random_share;
        req.start_delay_us = 0;
        req.extra_window_time_us = 0;

        dm_request_add(&req);
    }else{
        printk("%x %x %x\n", data[0], data[1], data[2]);
    }
    // Продолжаем обработку данных
    return BT_GATT_ITER_CONTINUE;
}

void led_on(void){
    printk("led on\n");
    dk_set_led(DK_LED1, 1);
    dk_set_led(DK_LED2, 1);
    dk_set_led(DK_LED3, 1);
    dk_set_led(DK_LED4, 1);
}

void led_off(void){
    printk("led off\n");
    dk_set_led(DK_LED1, 0);
    dk_set_led(DK_LED2, 0);
    dk_set_led(DK_LED3, 0);
    dk_set_led(DK_LED4, 0);
}

static void hogp_ready_cb(struct bt_hogp *hogp)
{
	k_work_submit(&hids_ready_work);
}

// Обработчик события "HIDS готов к работе"
static void hids_on_ready(struct k_work *work)/*переписать так, чтобы происходила обработка приходящих данных,
                                                но так чтобы при получении сигнала что-нибудь выводилось*/
{
    // Код ошибки
    int err;
    // Информация о репорте
    struct bt_hogp_rep_info *rep = NULL;

    // Выводим сообщение о том, что HIDS готов к работе
    printk("HIDS is ready to work\n");

    // Обрабатываем все доступные репорты
    while (NULL != (rep = bt_hogp_rep_next(&hogp, rep))) {
        // Если репорт является входным репортом
        if (bt_hogp_rep_type(rep) == BT_HIDS_REPORT_TYPE_INPUT) {
            // Выводим сообщение о том, что мы подписываемся на репорт
            printk("Subscribe to report id: %u\n", bt_hogp_rep_id(rep));
            // Подписываемся на репорт
            err = bt_hogp_rep_subscribe(&hogp, rep, hogp_notify_cb);
            // Если произошла ошибка, выводим сообщение об ошибке
            if (err) {
                printk("Subscribe error (%d)\n", err);
            }
        }
    }
}

static void hogp_prep_fail_cb(struct bt_hogp *hogp, int err)
{
	printk("ERROR: HIDS client preparation failed!\n");
}

static void hogp_pm_update_cb(struct bt_hogp *hogp)
{
	printk("Protocol mode updated: %s\n",
	      bt_hogp_pm_get(hogp) == BT_HIDS_PM_BOOT ?
	      "BOOT" : "REPORT");
}

// Параметры инициализации клиента HIDS
static const struct bt_hogp_init_params hogp_init_params = {
    // Функция обратного вызова, вызываемая при готовности клиента HIDS
    .ready_cb = hogp_ready_cb,
    // Функция обратного вызова, вызываемая при сбое подготовки клиента HIDS
    .prep_error_cb = hogp_prep_fail_cb,
    // Функция обратного вызова, вызываемая при обновлении протокола клиента HIDS
    .pm_update_cb = hogp_pm_update_cb
};


// Функция переключения режима работы устройства HOGP
static void button_bootmode(void)
{
    // Проверяем, готов ли устройство HOGP
    if (!bt_hogp_ready_check(&hogp)) {
        // Если устройство не готово, выводим сообщение и выходим
        printk("HID device not ready\n");
        return;
    }

    // Код ошибки
    int err;

    // Текущий режим работы устройства HOGP
    enum bt_hids_pm pm = bt_hogp_pm_get(&hogp);

    // Новый режим работы устройства HOGP
    enum bt_hids_pm new_pm = ((pm == BT_HIDS_PM_BOOT) ? BT_HIDS_PM_REPORT : BT_HIDS_PM_BOOT);

    // Выводим сообщение о новом режиме работы
    printk("Setting protocol mode: %s\n", (new_pm == BT_HIDS_PM_BOOT) ? "BOOT" : "REPORT");

    // Переключаем режим работы устройства HOGP
    err = bt_hogp_pm_write(&hogp, new_pm);

    // Если произошла ошибка, выводим сообщение об ошибке
    if (err) {
        printk("Cannot change protocol mode (err %d)\n", err);
    }
}

static void hidc_write_cb(struct bt_hogp *hidc,
			  struct bt_hogp_rep_info *rep,
			  uint8_t err)
{
	printk("Caps lock sent\n");
}

// Функция отправки команды Caps Lock на устройство HID
static void button_capslock(void)
{
    // Код ошибки
    int err;

    // Данные для отправки
    uint8_t data;

    // Проверяем, готов ли устройство HOGP
    if (!bt_hogp_ready_check(&hogp)) {
        // Если устройство не готово, выводим сообщение и выходим
        printk("HID device not ready\n");
        return;
    }

    // Проверяем, есть ли у устройства HID отчет о клавиатуре OUT
    if (!hogp.rep_boot.kbd_out) {
        // Если нет, выводим сообщение и выходим
        printk("HID device does not have Keyboard OUT report\n");
        return;
    }

    // Проверяем, находится ли устройство HOGP в режиме BOOT Report
    if (bt_hogp_pm_get(&hogp) != BT_HIDS_PM_BOOT) {
        // Если нет, выводим сообщение и выходим
        printk("This function works only in BOOT Report mode\n");
        return;
    }

    // Переключаем состояние Caps Lock
    capslock_state = capslock_state ? 0 : 1;

    // Подготавливаем данные для отправки
    data = capslock_state ? 0x02 : 0;

    // Отправляем данные на устройство HID
    err = bt_hogp_rep_write_wo_rsp(&hogp, hogp.rep_boot.kbd_out,
        &data, sizeof(data),
        hidc_write_cb);

    // Если произошла ошибка, выводим сообщение об ошибке
    if (err) {
        printk("Keyboard data write error (err: %d)\n", err);
        return;
    }

    // Выводим сообщение об отправке команды Caps Lock
    printk("Caps lock send (val: 0x%x)\n", data);
}


// Обратный вызов для чтения состояния Caps Lock
static uint8_t capslock_read_cb(struct bt_hogp *hogp,
        struct bt_hogp_rep_info *rep,
        uint8_t err,
        const uint8_t *data)
{
    // Если произошла ошибка, выводим сообщение об ошибке
    if (err) {
        printk("Capslock read error (err: %u)\n", err);
        return BT_GATT_ITER_STOP;
    }

    // Если нет данных, выводим сообщение
    if (!data) {
        printk("Capslock read - no data\n");
        return BT_GATT_ITER_STOP;
    }

    // Выводим информацию о полученных данных
    printk("Received data (size: %u, data[0]: 0x%x)\n",
            bt_hogp_rep_size(rep), data[0]);

    // Останавливаем итерацию
    return BT_GATT_ITER_STOP;
}


// Обратный вызов для записи состояния Caps Lock
static void capslock_write_cb(struct bt_hogp *hogp,
        struct bt_hogp_rep_info *rep,
        uint8_t err)
{
    int ret;

    // Выводим результат записи
    printk("Capslock write result: %u\n", err);

    // Читаем значение Caps Lock после записи
    ret = bt_hogp_rep_read(hogp, rep, capslock_read_cb);
    if (ret) {
        // Если произошла ошибка, выводим сообщение об ошибке
        printk("Cannot read capslock value (err: %d)\n", ret);
    }
}


// Функция отправки команды Caps Lock с использованием записи с ответом
static void button_capslock_rsp(void)
{
    // Проверяем, готов ли устройство HOGP
    if (!bt_hogp_ready_check(&hogp)) {
        printk("HID device not ready\n");
        return;
    }

    // Проверяем, есть ли у устройства HID отчет о клавиатуре OUT
    if (!hogp.rep_boot.kbd_out) {
        printk("HID device does not have Keyboard OUT report\n");
        return;
    }

    int err;
    uint8_t data;

    // Переключаем состояние Caps Lock
    capslock_state = capslock_state ? 0 : 1;
    data = capslock_state ? 0x02 : 0;

    // Отправляем команду Caps Lock с использованием записи с ответом
    err = bt_hogp_rep_write(&hogp, hogp.rep_boot.kbd_out, capslock_write_cb,
            &data, sizeof(data));
    if (err) {
        printk("Keyboard data write error (err: %d)\n", err);
        return;
    }

    // Выводим сообщение об отправке команды Caps Lock
    printk("Caps lock send using write with response (val: 0x%x)\n", data);
}


// Функция обработки ответа на запрос подтверждения числового соответствия
static void num_comp_reply(bool accept)
{
    // Если ответ положительный, подтверждаем парирование
    if (accept) {
        bt_conn_auth_passkey_confirm(auth_conn);
        printk("Numeric Match, conn %p\n", auth_conn);
    } else {
        // Если ответ отрицательный, отменяем парирование
        bt_conn_auth_cancel(auth_conn);
        printk("Numeric Reject, conn %p\n", auth_conn);
    }

    // Удаляем ссылку на соединение
    bt_conn_unref(auth_conn);
    auth_conn = NULL;
}


// Функция обработки нажатия кнопок
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    uint32_t button = button_state & has_changed;
    
    if (auth_conn) {
        num_comp_reply(true);
        return;
    }

    // Если есть соединение для парирования
    // if (auth_conn) {
    //     // Если кнопка "Принять", выполняем подтверждение парирования
    //     if (button & KEY_PAIRING_ACCEPT) {
    //         num_comp_reply(true);
    //     }

    //     // Если кнопка "Отклонить", выполняем отклонение парирования
    //     if (button & KEY_PAIRING_REJECT) {
    //         num_comp_reply(false);
    //     }
    //     return;
    // }
    // Если кнопка "Переключить режим загрузки", выполняем переключение режима загрузки
    if (button & KEY_BOOTMODE_MASK) {
        button_bootmode();
    }

    // Если кнопка "Включить/выключить Caps Lock", выполняем включение/выключение Caps Lock
    if (button & KEY_CAPSLOCK_MASK) {
        button_capslock();
    }

    // Если кнопка "Отправить команду Caps Lock с использованием записи с ответом", выполняем отправку команды Caps Lock
    if (button & KEY_CAPSLOCK_RSP_MASK) {
        button_capslock_rsp();
    }
}


// Функция отображения пароля для парирования устройства Bluetooth
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)    
{
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Выводим пароль для парирования устройства
    printk("Passkey for %s: %06u\n", addr, passkey);
}


// Функция подтверждения пароля для парирования устройства Bluetooth
static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    // Создаем ссылку на соединение
    auth_conn = bt_conn_ref(conn);

    // Преобразуем адрес устройства в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Выводим пароль для парирования устройства
    printk("Passkey for %s: %06u\n", addr, passkey);

    // Запрашиваем подтверждение пользователя
    printk("Press Button 1 to confirm, Button 2 to reject.\n");
}


// Функция обработки отмены парирования устройства Bluetooth
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Выводим сообщение об отмене парирования
    printk("Pairing cancelled: %s\n", addr);
}


// Функция обработки завершения процесса парирования устройства Bluetooth
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Выводим сообщение о завершении парирования
    printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}


// Функция обработки неудачи процесса парирования устройства Bluetooth
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    // Преобразуем адрес устройства в строковое представление
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Выводим сообщение о неудаче парирования
    printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
static void data_ready(struct dm_result *result)
{
	if (result->status) {
		peer_update(result);
	}
}

static struct dm_cb dm_cb = {
	.data_ready = data_ready,
};

int main(void)
{
    // Объявляем переменную для хранения ошибок
    int err;
    struct dm_init_param init_param;

    err = dk_leds_init();
    if (err) {
        // Вывод сообщения об ошибке при инициализации светодиодов
        printk("LEDs init failed (err %d)\n", err);
        return -1;
    }

/*
    // Инициализация узла связи
    err = peer_init();
    if (err) {
        printk("Peer init failed (err %d)\n", err);
        return 0;
    }
*/
    // Установка параметров инициализации модуля дистанционного управления
    init_param.cb = &dm_cb;

    // Инициализация модуля дистанционного управления
    err = dm_init(&init_param);
    if (err) {
        printk("Distance measurement init failed (err %d)\n", err);
        return 0;
    }

    // Выводим сообщение о начале работы примера Bluetooth Central HIDS
    printk("Starting Bluetooth Central HIDS example\n");
    
    // Инициализируем Bluetooth-стек с помощью функции bt_hogp_init
    // hogp - структура, содержащая параметры инициализации
    // hogp_init_params - структура, содержащая параметры инициализации
    bt_hogp_init(&hogp, &hogp_init_params);

    // Регистрируем обратные вызовы для авторизации соединения
    // conn_auth_callbacks - структура, содержащая функции обратных вызовов
    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        // Если регистрация обратных вызовов не удалась, выводим сообщение об ошибке
        printk("failed to register authorization callbacks.\n");
        // Возвращаем 0, чтобы указать на ошибку
        return 0;
    }

    // Регистрируем обратные вызовы для авторизации информации о соединении
    // conn_auth_info_callbacks - структура, содержащая функции обратных вызовов
    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err) {
        // Если регистрация обратных вызовов не удалась, выводим сообщение об ошибке
        printk("Failed to register authorization info callbacks.\n");
        // Возвращаем 0, чтобы указать на ошибку
        return 0;
    }

    // Инициализируем Bluetooth-адаптер
    err = bt_enable(NULL);
    if (err) {
        // Если инициализация не удалась, выводим сообщение об ошибке
        printk("Bluetooth init failed (err %d)\n", err);
        // Возвращаем 0, чтобы указать на ошибку
        return 0;
    }

    // Выводим сообщение о том, что Bluetooth инициализирован
    printk("Bluetooth initialized\n");

    // Если включена поддержка настроек (CONFIG_SETTINGS), загружаем настройки
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    // Инициализируем сканирование
    scan_init();

    // Инициализируем кнопки
    err = dk_buttons_init(button_handler);
    if (err) {
        // Если инициализация кнопок не удалась, выводим сообщение об ошибке
        printk("Failed to initialize buttons (err %d)\n", err);
        // Возвращаем 0, чтобы указать на ошибку
        return 0;
    }

    // Начинаем сканирование
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        // Если сканирование не удалось, выводим сообщение об ошибке
        printk("Scanning failed to start (err %d)\n", err);
        // Возвращаем 0, чтобы указать на ошибку
        return 0;
    }

    // Выводим сообщение о том, что сканирование успешно началось
    printk("Scanning successfully started\n");
    // Возвращаем 0, чтобы указать на успех
    return 0;
}
void test_run_cmd(void){
    printk("test run\n");
}

SHELL_CMD_REGISTER(run, NULL, "Run the test", test_run_cmd);
SHELL_CMD_REGISTER(on, NULL, "Run the test", led_on);
SHELL_CMD_REGISTER(off, NULL, "Run the test", led_off);
