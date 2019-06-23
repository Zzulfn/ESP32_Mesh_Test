// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_err.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"


#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "cmd_decl.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

#include "board.h"
#include "esp_fast_prov_operation.h"
#include "esp_fast_prov_client_model.h"
#include "esp_fast_prov_server_model.h"
#include "lwip/sockets.h"


#define TAG "BLE_MESH_WIFI_COEXIST_DEMO"


/*********************************Dick add ********************************************/
static void app_Wifi_handle(void);
static void app_initialise_wifi(void);
#define  EXAMPLE_WIFI_SSID "BLE_WIFI_TEST4"
#define  EXAMPLE_WIFI_PASS "12345678"
static void tcp_client_task(void *pvParameters);
static void WiFi_console_task(void *pvParameters);

#define HOST_IP_ADDR  "192.168.4.1"
#define HOST_PORT1   8083
   char payload[100] ;
//static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

#define CONFIG_EXAMPLE_IPV4  1
unsigned char ReceiveDataMark = 0;
int ReceiveData1 = 0;
int ReceiveDataNumFormBLEClient = 0;

char* St_BLE_Servr_Receive_Num =  "BLE_Server_Receive_Num:";
char* St_BLE_Client_Send_Num =  "BLE_Client_Send_Num:";
//char* St_FILLB = "  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB  ";

//char St_FILL[50] =  "AAAAAAAAA";


int sock = 0;

#define rx_buf_Size  1024
#define RxPackageSize 107

char rx_buffer[rx_buf_Size];
char rx_Temp_buffer[rx_buf_Size];
int  rx_head = 0;
int  rx_tail = 0;
int   WiFi_Staion_ReceiveNum = 0;
char* st_WiFi_Staion_ReceiveNum = "WiFi_Staion_ReceiveNum:";
SemaphoreHandle_t rx_buf_Mutex;




char Tx_buffer[1024];

#define  ReceivedData 1
#define  UnRecivedData 0
char WiFiReceiveDataMark = UnRecivedData;
char WiFiSendError = 0;

/*****************************************************************************/

extern struct _led_state led_state[3];
extern struct k_delayed_work send_self_prov_node_addr_timer;
extern bt_mesh_atomic_t fast_prov_cli_flags;

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };
static uint8_t prov_start_num = 0;
static bool prov_start = false;

static const esp_ble_mesh_client_op_pair_t fast_prov_cli_op_pair[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_SET,      ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_STATUS      },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_ADD,   ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_STATUS   },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR,     ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_ACK    },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_GET, ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_STATUS },
};

/* Configuration Client Model user_data */
esp_ble_mesh_client_t config_client;

/* Configuration Server Model user_data */
esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_DISABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/* Fast Prov Client Model user_data */
esp_ble_mesh_client_t fast_prov_client = {
    .op_pair_size = ARRAY_SIZE(fast_prov_cli_op_pair),
    .op_pair = fast_prov_cli_op_pair,
};

/* Fast Prov Server Model user_data */
example_fast_prov_server_t fast_prov_server = {
    .primary_role  = false,
    .max_node_num  = 6,
    .prov_node_cnt = 0x0,
    .unicast_min   = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .unicast_max   = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .unicast_cur   = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .unicast_step  = 0x0,
    .flags         = 0x0,
    .iv_index      = 0x0,
    .net_idx       = ESP_BLE_MESH_KEY_UNUSED,
    .app_idx       = ESP_BLE_MESH_KEY_UNUSED,
    .group_addr    = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .prim_prov_addr = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .match_len     = 0x0,
    .pend_act      = FAST_PROV_ACT_NONE,
    .state         = STATE_IDLE,
};

static esp_ble_mesh_model_op_t onoff_op[] = {
    { ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET,       0, NULL },
    { ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET,       2, NULL },
    { ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2, NULL },
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_pub_t onoff_pub = {
    .msg = NET_BUF_SIMPLE(2 + 1),
    .update = NULL,
};

static esp_ble_mesh_model_op_t fast_prov_srv_op[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_SET,      3,  NULL },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_ADD,   16, NULL },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR,     2,  NULL },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_GET, 0,  NULL },
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_op_t fast_prov_cli_op[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_STATUS,    1, NULL },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_STATUS, 2, NULL },
    { ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_ACK,  0, NULL },
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_SIG_MODEL(ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, onoff_op,
    &onoff_pub, &led_state[1]),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_FAST_PROV_SRV,
    fast_prov_srv_op, NULL, &fast_prov_server),
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_FAST_PROV_CLI,
    fast_prov_cli_op, NULL, &fast_prov_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t comp = {
    .cid           = CID_ESP,
    .elements      = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t prov = {
    .uuid                = dev_uuid,
    .output_size         = 0,
    .output_actions      = 0,
    .prov_attention      = 0x00,
    .prov_algorithm      = 0x00,
    .prov_pub_key_oob    = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags               = 0x00,
    .iv_index            = 0x00,
};

esp_ble_mesh_model_t DickTest_model;
esp_ble_mesh_msg_ctx_t DickTest_ctx;
unsigned char TestMark = 0;
	
static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT |BIT0;

    ESP_LOGI(TAG, "Waiting for AP connection...");
	while(wifi_event_group == NULL){
		vTaskDelay(100);
	}
	ESP_LOGI(TAG, "Waiting for WiFi_event_group");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}

static void gen_onoff_get_handler(esp_ble_mesh_model_t *model,
                                  esp_ble_mesh_msg_ctx_t *ctx,
                                  uint16_t len, uint8_t *data)
{
    struct _led_state *led = NULL;
    uint8_t send_data;
    esp_err_t err;

    led = (struct _led_state *)model->user_data;
    if (!led) {
        ESP_LOGE(TAG, "%s: Failed to get generic onoff server model user_data", __func__);
        return;
    }

    send_data = led->current;
    ESP_LOGI(TAG, "%s, addr 0x%04x onoff 0x%02x", __func__, model->element->element_addr, led->current);
//	ctx->addr = 4;//Dick add
   if(TestMark == 0)
   	{   
   	    TestMark =1;
 	    ESP_LOGI(TAG, "Dick Test TestMark:%d",TestMark);
		memcpy((unsigned char*)&DickTest_model,(unsigned char *)model,sizeof(esp_ble_mesh_model_t));
		memcpy((unsigned char*)&DickTest_ctx,(unsigned char *)ctx,sizeof(esp_ble_mesh_msg_ctx_t));
   	}
    /* Sends Generic OnOff Status as a reponse to Generic OnOff Get */
    err = esp_ble_mesh_server_model_send_msg(model, ctx,  ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET, sizeof(send_data), &send_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to send Generic OnOff Status message", __func__);
        return;
    }

    /* When the node receives the first Generic OnOff Get/Set/Set Unack message, it will
     * start the timer used to disable fast provisioning functionality.
     */
    if (!bt_mesh_atomic_test_and_set_bit(fast_prov_server.srv_flags, DISABLE_FAST_PROV_START)) {
        k_delayed_work_submit(&fast_prov_server.disable_fast_prov_timer, DISABLE_FAST_PROV_TIMEOUT);
    }
}


static void gen_onoff_set_unack_handler(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx, uint16_t len, uint8_t *data)
{
    struct _led_state *led = NULL;

    led = (struct _led_state *)model->user_data;
    if (!led) {
        ESP_LOGE(TAG, "%s: Failed to get generic onoff server model user_data", __func__);
        return;
    }
    ReceiveDataMark = 1;
	ReceiveDataNumFormBLEClient ++;
    led->current = data[0];
	ReceiveData1 =led->current;
    ESP_LOGI(TAG, "Dick     ReceiveDataMark = %d addr 0x%02x onoff 0x%02x", ReceiveDataMark,model->element->element_addr, led->current);

    if (led_action_task_post(led, portMAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to post led action to queue", __func__);
        return;
    }

    /* When the node receives the first Generic OnOff Get/Set/Set Unack message, it will
     * start the timer used to disable fast provisioning functionality.
     */
    if (!bt_mesh_atomic_test_and_set_bit(fast_prov_server.srv_flags, DISABLE_FAST_PROV_START)) {
        k_delayed_work_submit(&fast_prov_server.disable_fast_prov_timer, DISABLE_FAST_PROV_TIMEOUT);
    }
}

static void gen_onoff_set_handler(esp_ble_mesh_model_t *model,
                                  esp_ble_mesh_msg_ctx_t *ctx,
                                  uint16_t len, uint8_t *data)
{
    gen_onoff_set_unack_handler(model, ctx, len, data);
  //  gen_onoff_get_handler(model, ctx, len, data); //把数据有返回
}


static void node_prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{

    ESP_LOGI(TAG, "net_idx: 0x%04x, unicast_addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08x", flags, iv_index);
    board_prov_complete();
    /* Updates the net_idx used by Fast Prov Server model, and it can also
     * be updated if the Fast Prov Info Set message contains a valid one.
     */
    fast_prov_server.net_idx = net_idx;
	
}

static void provisioner_prov_link_open(esp_ble_mesh_prov_bearer_t bearer)
{
    ESP_LOGI(TAG, "%s: bearer %s", __func__, bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
}

static void provisioner_prov_link_close(esp_ble_mesh_prov_bearer_t bearer, uint8_t reason)
{
    ESP_LOGI(TAG, "%s: bearer %s, reason 0x%02x", __func__,
             bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", reason);
    if (prov_start_num) {
        prov_start_num--;
    }
}

static void provisioner_prov_complete(int node_idx, const uint8_t uuid[16], uint16_t unicast_addr,
                                      uint8_t element_num, uint16_t net_idx)
{
    example_node_info_t *node = NULL;
    esp_err_t err;

    if (example_is_node_exist(uuid) == false) {
        fast_prov_server.prov_node_cnt++;
    }

    /* Sets node info */
    err = example_store_node_info(uuid, unicast_addr, element_num, net_idx,
                                  fast_prov_server.app_idx, LED_OFF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to set node info", __func__);
        return;
    }

    /* Gets node info */
    node = example_get_node_info(unicast_addr);
    if (!node) {
        ESP_LOGE(TAG, "%s: Failed to get node info", __func__);
        return;
    }

    if (fast_prov_server.primary_role == true) {
        /* If the Provisioner is the primary one (i.e. provisioned by the phone), it shall
         * store self-provisioned node addresses;
         * If the node_addr_cnt configured by the phone is small than or equal to the
         * maximum number of nodes it can provision, it shall reset the timer which is used
         * to send all node addresses to the phone.
         */
        err = example_store_remote_node_address(unicast_addr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to store node address 0x%04x", __func__, unicast_addr);
            return;
        }
        if (fast_prov_server.node_addr_cnt <= fast_prov_server.max_node_num) {
            if (bt_mesh_atomic_test_and_clear_bit(fast_prov_server.srv_flags, SEND_ALL_NODE_ADDR_START)) {
                k_delayed_work_cancel(&fast_prov_server.send_all_node_addr_timer);
            }
            if (!bt_mesh_atomic_test_and_set_bit(fast_prov_server.srv_flags, SEND_ALL_NODE_ADDR_START)) {
                k_delayed_work_submit(&fast_prov_server.send_all_node_addr_timer, SEND_ALL_NODE_ADDR_TIMEOUT);
            }
        }
    } else {
        /* When a device is provisioned, the non-primary Provisioner shall reset the timer
         * which is used to send node addresses to the primary Provisioner.
         */
        if (bt_mesh_atomic_test_and_clear_bit(&fast_prov_cli_flags, SEND_SELF_PROV_NODE_ADDR_START)) {
            k_delayed_work_cancel(&send_self_prov_node_addr_timer);
        }
        if (!bt_mesh_atomic_test_and_set_bit(&fast_prov_cli_flags, SEND_SELF_PROV_NODE_ADDR_START)) {
            k_delayed_work_submit(&send_self_prov_node_addr_timer, SEND_SELF_PROV_NODE_ADDR_TIMEOUT);
        }
    }

    if (bt_mesh_atomic_test_bit(fast_prov_server.srv_flags, DISABLE_FAST_PROV_START)) {
        /* When a device is provisioned, and the stop_prov flag of the Provisioner has been
         * set, the Provisioner shall reset the timer which is used to stop the provisioner
         * functionality.
         */
        k_delayed_work_cancel(&fast_prov_server.disable_fast_prov_timer);
        k_delayed_work_submit(&fast_prov_server.disable_fast_prov_timer, DISABLE_FAST_PROV_TIMEOUT);
    }

    /* The Provisioner will send Config AppKey Add to the node. */
    example_msg_common_info_t info = {
        .net_idx = node->net_idx,
        .app_idx = node->app_idx,
        .dst = node->unicast_addr,
        .timeout = 0,
        .role = ROLE_FAST_PROV,
    };
    err = example_send_config_appkey_add(config_client.model, &info, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to send Config AppKey Add message", __func__);
        return;
    }
}

static void example_recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[ESP_BD_ADDR_LEN],
                                        esp_ble_addr_type_t addr_type, uint16_t oob_info,
                                        uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    esp_ble_mesh_dev_add_flag_t flag;
    esp_err_t err;

    /* In Fast Provisioning, the Provisioner should only use PB-ADV to provision devices. */
    if (prov_start && (bearer & ESP_BLE_MESH_PROV_ADV)) {
        /* Checks if the device is a reprovisioned one. */
        if (example_is_node_exist(dev_uuid) == false) {
            if ((prov_start_num >= fast_prov_server.max_node_num) ||
                    (fast_prov_server.prov_node_cnt >= fast_prov_server.max_node_num)) {
                return;
            }
        }

        add_dev.addr_type = (uint8_t)addr_type;
        add_dev.oob_info = oob_info;
        add_dev.bearer = (uint8_t)bearer;
        memcpy(add_dev.uuid, dev_uuid, 16);
        memcpy(add_dev.addr, addr, ESP_BD_ADDR_LEN);
        flag = ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG;
        err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev, flag);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to start provisioning device", __func__);
            return;
        }

        /* If adding unprovisioned device successfully, increase prov_start_num */
        prov_start_num++;
    }

    return;
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
        esp_ble_mesh_prov_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code: %d",
                 param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code: %d",
                 param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer: %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer: %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        node_prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROXY_GATT_DISABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROXY_GATT_DISABLE_COMP_EVT");
        if (fast_prov_server.primary_role == true) {
            config_server.relay = ESP_BLE_MESH_RELAY_DISABLED;
        }
        prov_start = true;
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        example_recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                                    param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                                    param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT");
        provisioner_prov_link_open(param->provisioner_prov_link_open.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT");
        provisioner_prov_link_close(param->provisioner_prov_link_close.bearer,
                                    param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT");
        provisioner_prov_complete(param->provisioner_prov_complete.node_idx,
                                  param->provisioner_prov_complete.device_uuid,
                                  param->provisioner_prov_complete.unicast_addr,
                                  param->provisioner_prov_complete.element_num,
                                  param->provisioner_prov_complete.netkey_idx);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code: %d",
                 param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code: %d",
                 param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code: %d",
                 param->provisioner_set_node_name_comp.err_code);
        break;
    case ESP_BLE_MESH_SET_FAST_PROV_INFO_COMP_EVT: {
        ESP_LOGI(TAG, "ESP_BLE_MESH_SET_FAST_PROV_INFO_COMP_EVT");
       #if 0
	   ESP_LOGI(TAG, "status_unicast: 0x%02x, status_net_idx: 0x%02x, status_match 0x%02x",
                 param->set_fast_prov_info_comp.status_unicast,
                 param->set_fast_prov_info_comp.status_net_idx,
                 param->set_fast_prov_info_comp.status_match);

		#endif
        err = example_handle_fast_prov_info_set_comp_evt(fast_prov_server.model,
                param->set_fast_prov_info_comp.status_unicast,
                param->set_fast_prov_info_comp.status_net_idx,
                param->set_fast_prov_info_comp.status_match);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to handle Fast Prov Info Set complete event", __func__);
            return;
        }
        break;
    }
    case ESP_BLE_MESH_SET_FAST_PROV_ACTION_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_SET_FAST_PROV_ACTION_COMP_EVT, status_action 0x%02x",
                 param->set_fast_prov_action_comp.status_action);
        err = example_handle_fast_prov_action_set_comp_evt(fast_prov_server.model,
                param->set_fast_prov_action_comp.status_action);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: Failed to handle Fast Prov Action Set complete event", __func__);
            return;
        }
        break;
    default:
        break;
    }

    return;
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
        esp_ble_mesh_model_cb_param_t *param)
{
    uint32_t opcode;
    esp_err_t err;
	
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT: {
        if (!param->model_operation.model || !param->model_operation.model->op ||
                !param->model_operation.ctx) {
            ESP_LOGE(TAG, "%s: model_operation parameter is NULL", __func__);
            return;
        }
        opcode = param->model_operation.opcode;
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
            gen_onoff_set_unack_handler(param->model_operation.model, param->model_operation.ctx,
                                  param->model_operation.length, param->model_operation.msg);
            break;
			
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
			
			#if 1
            gen_onoff_set_unack_handler(param->model_operation.model, param->model_operation.ctx,
                                        param->model_operation.length, param->model_operation.msg);
			#endif 
            break;
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_SET:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_ADD:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_GET: {
            ESP_LOGI(TAG, "%s: Fast prov server receives msg, opcode 0x%04x", __func__, opcode);
            struct net_buf_simple buf = {
                .len = param->model_operation.length,
                .data = param->model_operation.msg,
            };
            err = example_fast_prov_server_recv_msg(param->model_operation.model,
                                                    param->model_operation.ctx, &buf);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s: Failed to handle fast prov client message", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_STATUS:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_STATUS:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_ACK: {
            ESP_LOGI(TAG, "%s: Fast prov client receives msg, opcode 0x%04x", __func__, opcode);
            err = example_fast_prov_client_recv_status(param->model_operation.model,
                    param->model_operation.ctx,
                    param->model_operation.length,
                    param->model_operation.msg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s: Failed to handle fast prov server message", __func__);
                return;
            }
            break;
        }
        default:
            ESP_LOGI(TAG, "%s: opcode 0x%04x", __func__, param->model_operation.opcode);
            break;
        }
        break;
    }
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_SEND_COMP_EVT, err_code %d", param->model_send_comp.err_code);
        switch (param->model_send_comp.opcode) {
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_INFO_STATUS:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NET_KEY_STATUS:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_ACK:
        case ESP_BLE_MESH_VND_MODEL_OP_FAST_PROV_NODE_ADDR_STATUS:
            err = example_handle_fast_prov_status_send_comp_evt(param->model_send_comp.err_code,
                    param->model_send_comp.opcode,
                    param->model_send_comp.model,
                    param->model_send_comp.ctx);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s: Failed to handle fast prov status send complete event", __func__);
                return;
            }
            break;
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT, err_code %d",
                 param->model_publish_comp.err_code);
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_CLIENT_RECV_PUBLISH_MSG_EVT, opcode 0x%04x",
                 param->client_recv_publish_msg.opcode);
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT, opcode 0x%04x, dst 0x%04x",
                 param->client_send_timeout.opcode, param->client_send_timeout.ctx->addr);
        err = example_fast_prov_client_recv_timeout(param->client_send_timeout.opcode,
                param->client_send_timeout.model,
                param->client_send_timeout.ctx);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: Faield to resend fast prov client message", __func__);
            return;
        }
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
        esp_ble_mesh_cfg_client_cb_param_t *param)
{
    example_node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t address;
    esp_err_t err;

    ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x",
             __func__, param->error_code, event, param->params->ctx.addr);

    opcode = param->params->opcode;
    address = param->params->ctx.addr;

    node = example_get_node_info(address);
    if (!node) {
        ESP_LOGE(TAG, "%s: Failed to get node info", __func__);
        return;
    }

    if (param->error_code) {
        ESP_LOGE(TAG, "Failed to send config client message, opcode: 0x%04x", opcode);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: {
            example_fast_prov_info_set_t set = {0};
            if (node->reprov == false) {
                /* After sending Config AppKey Add successfully, start to send Fast Prov Info Set */
                if (fast_prov_server.unicast_cur >= fast_prov_server.unicast_max) {
                    /* TODO:
                     * 1. If unicast_cur is >= unicast_max, we can also send the message to enable
                     * the Provisioner functionality on the node, and need to add another vendor
                     * message used by the node to require a new unicast address range from primary
                     * Provisioner, and before get the correct response, the node should pend
                     * the fast provisioning functionality.
                     * 2. Currently if address is not enough, the Provisioner will only add the group
                     * address to the node.
                     */
                    ESP_LOGW(TAG, "%s: Not enough address to be assigned", __func__);
                    node->lack_of_addr = true;
                } else {
                    /* Send fast_prov_info_set message to node */
                    node->lack_of_addr = false;
                    node->unicast_min = fast_prov_server.unicast_cur;
                    if (fast_prov_server.unicast_cur + fast_prov_server.unicast_step >= fast_prov_server.unicast_max) {
                        node->unicast_max = fast_prov_server.unicast_max;
                    } else {
                        node->unicast_max = fast_prov_server.unicast_cur + fast_prov_server.unicast_step;
                    }
                    node->flags      = fast_prov_server.flags;
                    node->iv_index   = fast_prov_server.iv_index;
                    node->fp_net_idx = fast_prov_server.net_idx;
                    node->group_addr = fast_prov_server.group_addr;
                    node->prov_addr  = fast_prov_server.prim_prov_addr;
                    node->match_len  = fast_prov_server.match_len;
                    memcpy(node->match_val, fast_prov_server.match_val, fast_prov_server.match_len);
                    node->action = FAST_PROV_ACT_ENTER;
                    fast_prov_server.unicast_cur = node->unicast_max + 1;
                }
            }
            if (node->lack_of_addr == false) {
                set.ctx_flags = 0x03FE;
                memcpy(&set.unicast_min, &node->unicast_min,
                       sizeof(example_node_info_t) - offsetof(example_node_info_t, unicast_min));
            } else {
                set.ctx_flags  = BIT(6);
                set.group_addr = fast_prov_server.group_addr;
            }
            example_msg_common_info_t info = {
                .net_idx = node->net_idx,
                .app_idx = node->app_idx,
                .dst = node->unicast_addr,
                .timeout = 0,
                .role = ROLE_FAST_PROV,
            };
            err = example_send_fast_prov_info_set(fast_prov_client.model, &info, &set);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s: Failed to send Fast Prov Info Set message", __func__);
                return;
            }
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (opcode) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: {
            example_msg_common_info_t info = {
                .net_idx = node->net_idx,
                .app_idx = node->app_idx,
                .dst = node->unicast_addr,
                .timeout = 0,
                .role = ROLE_FAST_PROV,
            };
            err = example_send_config_appkey_add(config_client.model, &info, NULL);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s: Failed to send Config AppKey Add message", __func__);
                return;
            }
            break;
        }
        default:
            break;
        }
        break;
    default:
        return;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
        esp_ble_mesh_cfg_server_cb_param_t *param)
{
    esp_err_t err;

    ESP_LOGI(TAG, "%s, event = 0x%02x, opcode = 0x%04x, addr: 0x%04x",
             __func__, event, param->ctx.recv_op, param->ctx.addr);

    switch (event) {
    case ESP_BLE_MESH_CFG_SERVER_RECV_MSG_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "Config Server get Config AppKey Add");
            err = example_handle_config_app_key_add_evt(param->status_cb.app_key_add.app_idx);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "%s: Failed to bind app_idx 0x%04x with non-config models",
                         __func__, param->status_cb.app_key_add.app_idx);
                return;
            }
            break;
        default:
            break;
        }
        break;
    default:
        return;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    /* First two bytes of device uuid is compared with match value by Provisioner */
    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), 6);

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);

    err = esp_ble_mesh_init(&prov, &comp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to initialize BLE Mesh", __func__);
        return err;
    }

    err = example_fast_prov_server_init(&vnd_models[0]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to initialize fast prov server model", __func__);
        return err;
    }

    err = esp_ble_mesh_client_model_init(&vnd_models[1]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to initialize fast prov client model", __func__);
        return err;
    }

    k_delayed_work_init(&send_self_prov_node_addr_timer, example_send_self_prov_node_addr);

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed to enable node provisioning", __func__);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Wi-Fi Coexist Node initialized");

    board_led_operation(LED_B, LED_ON);

    return ESP_OK;
}

static esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return ret;
    }

    return ret;
}

#define WIFI_CONNECTED_BIT BIT0

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}


static void initialize_console()
{
    /* Disable buffering on stdin and stdout */
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_CONSOLE_UART_NUM,
                                         256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 32,
        .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback *) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);
}

static void app_initialise_wifi(void)
{ 

	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	wifi_config_t wifi_config = {
			.sta = {
				.ssid = EXAMPLE_WIFI_SSID,
				.password = EXAMPLE_WIFI_PASS,
			},
		};
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK( esp_wifi_start() );
	

}


static void wifi_console_init(void)
{   
   
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI("Dick", "%s",__func__);
    initialise_wifi();
    initialize_console();

    /* Register commands */
    esp_console_register_help_command();
//	esp_console_register_SetServer_command();
    register_wifi();
    
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char *prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;
	
    printf("\n ==================================================\n");
    printf(" |       Steps to test WiFi throughput            |\n");
    printf(" |                                                |\n");
    printf(" |  1. Print 'help' to gain overview of commands  |\n");
    printf(" |  2. Configure device to station or soft-AP     |\n");
    printf(" |  3. Setup WiFi connection                      |\n");
    printf(" |  4. Run iperf to test UDP/TCP RX/TX throughput |\n");
    printf(" |                                                |\n");
    printf(" =================================================\n\n");
	
    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "esp32> ";
#endif //CONFIG_LOG_COLORS
    }
	
    /* Main loop */
    while (true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char *line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x\n", ret);
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }

    return;

}
void tcp_sentData_task(void){

	
static int SentDelayTimesNum = 0;
int Result = 0;
int err  = 0;

	while(1){
		
		vTaskDelay(10);  //waite the sock successful
		
		while(sock > 0){
		
			SentDelayTimesNum++;
				
		    if(SentDelayTimesNum >= 400){
		
				SentDelayTimesNum = 0;
				
			
			    if(ReceiveDataMark == 1){
					
					ReceiveDataMark = 0;
					
					sprintf(payload, "%s%d  %s  %s%d", St_BLE_Servr_Receive_Num, ReceiveDataNumFormBLEClient, St_FILL,St_BLE_Client_Send_Num,ReceiveData1);

					ESP_LOGI(TAG, "Dick WiFi Send data to Server:%s",payload);
		            err = send(sock, payload, strlen(payload), 0);
		            if (err < 0) {
						WiFiSendError = err;
		                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
		                break;
		            }

			    }
		    }
			
			Result  =  xSemaphoreTakeRecursive( rx_buf_Mutex,portMAX_DELAY);
			  ESP_LOGE(TAG, "Dick Take recursive rx_head:%d  rx_tail:%d",rx_head,rx_tail);
			   vTaskDelay(10);
				while(rx_head < rx_tail){
					if(rx_buffer[rx_head] == 'h'){
						
					//	ESP_LOGE(TAG, "Dick find h rx_head:%d",rx_head);
						if(rx_head +RxPackageSize <= rx_tail){
							
					//		ESP_LOGE(TAG, "rx_head +RxPackageSize <= rx_tail ,rx_buffer[rx_head+RxPackageSize-1]%c:",rx_buffer[rx_head+RxPackageSize-1]);
							if(rx_buffer[rx_head+RxPackageSize-1] == 'l'){
						//		ESP_LOGE(TAG, "rx_buffer[rx_head+RxPackageSize] == r");
								WiFi_Staion_ReceiveNum++;
								memcpy(&rx_Temp_buffer[0],&rx_buffer[rx_head],RxPackageSize);
								memset(& Tx_buffer, 0, sizeof(Tx_buffer));
							    rx_Temp_buffer[RxPackageSize] = '\0';
								sprintf(Tx_buffer, "%s%d%s",st_WiFi_Staion_ReceiveNum, WiFi_Staion_ReceiveNum,rx_Temp_buffer);
								int err = send(sock, Tx_buffer, strlen(Tx_buffer), 0);
								if (err < 0) {
									ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
								//	WiFiSendError = err;
									break;
								}
								
							}
							rx_head = rx_head+RxPackageSize;
							break;
						}
						else{
							break;
						}
					}
					else{
						rx_head++;
					}
				}

				//移动数据
				memset(&rx_Temp_buffer[0], 0x00, sizeof(rx_Temp_buffer));
				MEMCPY(&rx_Temp_buffer[0],  &rx_buffer[rx_head], rx_buf_Size-rx_head);
				memset(&rx_buffer[0], 0x00, sizeof(rx_buffer));	
				MEMCPY(&rx_buffer[0],  &rx_Temp_buffer[0], rx_buf_Size);
				rx_tail = rx_tail - rx_head;
				rx_head = 0;
				
				xSemaphoreGiveRecursive( rx_buf_Mutex);

				if(err < 0 ){
					//WiFiSendError = err;
					break;
				}
						
		    }
			
		vTaskDelay(5);
	}
}



static void WiFi_console_task(void *pvParameters){

    ESP_LOGI("Dick", "%s", __func__);
	wifi_console_init(); //config the WiFi SSID and PassKey 
}





static void tcp_client_task(void *pvParameters)
{

    char addr_str[128];
	char temp_buf[1024] = {0};
    int addr_family;
    int ip_protocol;
	int ServerPort = 8080;
	static int SentDelayTimesNum  = 0;
	char Temp = 0;
	int Result = 0;

	rx_buf_Mutex = xSemaphoreCreateMutex();
	
//	wait_for_ip(); //Wait  for connecting  AP OK 
	xTaskCreate(tcp_sentData_task, "tcp_sentData_task", 4096, NULL, 5, NULL);  
	
	Temp = 'A'+(HOST_PORT1-8080);
	memset((unsigned char*)&St_FILL,Temp,sizeof(St_FILL));
	
    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr("192.168.4.1");
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(HOST_PORT1);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton("192.168.4.1", &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(HOST_PORT1);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
			vTaskDelay(1000);
		   if (sock != -1) {
	            ESP_LOGE(TAG, "Shutting down socket and restarting...");
	            shutdown(sock, 0);
	            close(sock);
	        }
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");
		ReceiveDataMark = UnRecivedData;
		WiFiSendError = 0;
		
        while (1) {

		    if (WiFiSendError !=  0)
				break;

		    vTaskDelay(10 / portTICK_PERIOD_MS);

#if 0
		  
			SentDelayTimesNum++;
				
		    if(SentDelayTimesNum == 100){

				SentDelayTimesNum = 0;
			    if(ReceiveDataMark == 1){
					
					ReceiveDataMark = 1;
					
					sprintf(payload, "%s%d  %s  %s%d", St_BLE_Servr_Receive_Num, ReceiveDataNumFormBLEClient, St_FILL,St_BLE_Client_Send_Num,ReceiveData1);

					ESP_LOGI(TAG, "Dick WiFi Send data to Server:%s",payload);
		            int err = send(sock, payload, strlen(payload), 0);
		            if (err < 0) {
		                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
		                break;
		            }

			    }
		    }
#endif 
			
		#if 1 
			if(WiFiReceiveDataMark ==UnRecivedData) { 
				
			    ///	
				//untile  handle the data received not Receive other Data 
	            int len = recv(sock, temp_buf, sizeof(temp_buf) - 1, 0);
	            // Error occured during receiving
	            if (len < 0) {
	                ESP_LOGE(TAG, "recv failed: errno %d", errno);
	                break;
	            }
	            // Data received
	            else if(len > 0) {
					
                        Result  =  xSemaphoreTakeRecursive( rx_buf_Mutex, portMAX_DELAY);
						if(rx_tail +len < rx_buf_Size && Result == pdTRUE){
							memcpy(&rx_buffer[rx_tail],&temp_buf[0],len);
							rx_tail = rx_tail +len;
						}
						ESP_LOGI(TAG, "recv data:%s", rx_buffer);
						xSemaphoreGiveRecursive( rx_buf_Mutex);
	             }

			}
         #endif
            
		//	ESP_LOGI(TAG,"vtaskdelay(2000)ReceiveDataMark ：%d",ReceiveDataMark);
			
        }
       
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}




void app_Wifi_handle(void){

   esp_err_t err = nvs_flash_init();
       if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	ESP_ERROR_CHECK( err );
	   
	app_initialise_wifi();
    //xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, NULL, 5, NULL, 1);
}

void app_main(void)
{
    esp_err_t err;
	
    ESP_LOGI(TAG, "Initializing...");
    
    err = board_init();
    if (err) {
        ESP_LOGE(TAG, "board_init failed (err %d)", err);
        return;
    }
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }
    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
        return;
    }
	app_initialise_wifi();
	wait_for_ip();
	xTaskCreate(tcp_client_task, "tcp_server_task", 2048+4096, NULL, 5, NULL);
	
}
