#include "indicator_matter.h"
#include "indicator_storage.h"
#include "matter_config.h"

#include <atomic>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/ESP32/ESP32Utils.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
using namespace esp_matter::cluster::basic_information::attribute;

namespace {

const char *TAG = "matter";
uint16_t temperature_endpoint_id = 0;
uint16_t humidity_endpoint_id = 0;
uint16_t extended_color_light_endpoint_id = 0;
uint16_t door_lock_endpoint_id = 0;
uint16_t extended_color_light_endpoint2_id = 0;
std::atomic_bool __g_matter_connected_flag = false;
bool __g_ip_connected_flag = false;
constexpr auto k_timeout_seconds = 300;
esp_timer_handle_t   matter_sensor_timer_handle;

struct Sensors {
  std::atomic<int16_t> temperature = 0;
  std::atomic<int16_t> humidity = 0;
};

Sensors __g_sensor_values;

void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "IP Interface Address Changed");
        if (event->InterfaceIpAddressChanged.Type == chip::DeviceLayer::InterfaceIpChangeType::kIpV6_Assigned ||
                        event->InterfaceIpAddressChanged.Type == chip::DeviceLayer::InterfaceIpChangeType::kIpV4_Assigned) {
            __g_ip_connected_flag = true;
            
            struct view_data_wifi_st st;
            st.rssi = -50;
            st.is_connected = true;
            esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_WIFI_ST, &st, sizeof(struct view_data_wifi_st ), portMAX_DELAY);

            __g_matter_connected_flag = true;
            uint8_t screen = SCREEN_DASHBOARD;
            ESP_ERROR_CHECK(esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_SCREEN_START, &screen, sizeof(screen), portMAX_DELAY));
        }
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete: {
        ESP_LOGI(TAG, "Connected");
        __g_matter_connected_flag = true;
        break;
    }
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
    {
        ESP_LOGI(TAG, "Commissioning window closed");
        break;
    }
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
    {
        ESP_LOGI(TAG, "Fabric removed successfully");
        __g_matter_connected_flag = false;
        chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
        constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
        if (!commissionMgr.IsCommissioningWindowOpen())
        {
            CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                            chip::CommissioningWindowAdvertisement::kDnssdOnly);
            if (err != CHIP_NO_ERROR)
            {
                ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
            }
        }

        ESP_LOGI(TAG, "Beginning Matter Provisioning");
        uint8_t screen = SCREEN_MATTER_CONFIG;
        ESP_ERROR_CHECK(esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_SCREEN_START, &screen, sizeof(screen), portMAX_DELAY));
        break;
    }
    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
    {
        ESP_LOGI(TAG, "Fabric is committed");

        __g_matter_connected_flag = true;
        uint8_t screen = SCREEN_DASHBOARD;
        ESP_ERROR_CHECK(esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_SCREEN_START, &screen, sizeof(screen), portMAX_DELAY));
        break;
    }
    default:
        break;
    }
}

esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    if (type == PRE_UPDATE) {
        esp_err_t err = ESP_OK;     
        if (endpoint_id == extended_color_light_endpoint_id) {
            if (cluster_id == OnOff::Id) {
                if (attribute_id == OnOff::Attributes::OnOff::Id) {
                    struct view_data_matter_dashboard_data data;
                    data.dashboard_data_type = DASHBOARD_DATA_BUTTON1;
                    data.value = (int)val->val.b;
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);
                }
            } else if (cluster_id == LevelControl::Id) {
                if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                    struct view_data_matter_dashboard_data data;
                    data.dashboard_data_type = DASHBOARD_DATA_ARC;
                    data.value = REMAP_TO_RANGE(val->val.u8, MATTER_BRIGHTNESS, STANDARD_BRIGHTNESS);
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);                    

                    struct view_data_matter_dashboard_data on_data;
                    on_data.dashboard_data_type = DASHBOARD_DATA_BUTTON1;
                    if (val->val.u8 > 0) {
                        on_data.value = 1;
                    } else {
                        on_data.value = 0;
                    }
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &on_data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);
                }
            }
        } else if (endpoint_id == extended_color_light_endpoint2_id) {
            if (cluster_id == OnOff::Id) {
                if (attribute_id == OnOff::Attributes::OnOff::Id) {
                    struct view_data_matter_dashboard_data data;
                    data.dashboard_data_type = DASHBOARD_DATA_BUTTON2;
                    data.value = (int)val->val.b;
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);
                    
                }
            } else if (cluster_id == LevelControl::Id) {
                if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                    struct view_data_matter_dashboard_data data;
                    data.dashboard_data_type = DASHBOARD_DATA_SLIDER;
                    data.value = REMAP_TO_RANGE(val->val.u8, MATTER_BRIGHTNESS, STANDARD_BRIGHTNESS);
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);                                      

                    struct view_data_matter_dashboard_data on_data;
                    on_data.dashboard_data_type = DASHBOARD_DATA_BUTTON2;
                    if (val->val.u8 > 0) {
                        on_data.value = 1;
                    } else {
                        on_data.value = 0;
                    }
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &on_data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);
                }
            }
        } else if (endpoint_id == door_lock_endpoint_id) {
            if (cluster_id == DoorLock::Id) {
                if (attribute_id == DoorLock::Attributes::LockState::Id) {
                    struct view_data_matter_dashboard_data data;
                    data.dashboard_data_type = DASHBOARD_DATA_SWITCH;
                    data.value = (int)val->val.b;
                    esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_MATTER_SET_DASHBOARD_DATA, \
                        &data, sizeof(struct view_data_matter_dashboard_data ), portMAX_DELAY);
                }
            }
        }
        return err;
    }

    return err;
}

// Reports the given value to the given attribute if Matter is connected.
void update_attribute(node_t *node, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
                      esp_matter_attr_val_t value, const char* name) {
    const char* action_name = "not logging since Matter is not connected";
    if (__g_matter_connected_flag.load()) {
        action_name = "updated matter value";
        endpoint_t *endpoint = endpoint::get(node, endpoint_id);
        cluster_t *cluster = cluster::get(endpoint, cluster_id);
        attribute_t *attribute = attribute::get(cluster, attribute_id);
        attribute::update(endpoint_id, cluster_id, attribute_id, &value);
    }
    if (value.type == ESP_MATTER_VAL_TYPE_INT16 || value.type == ESP_MATTER_VAL_TYPE_NULLABLE_INT16) {
        ESP_LOGI(TAG, "%s: %s. Value is %" PRIi16, name, action_name, value.val.i16);
    } else if (value.type == ESP_MATTER_VAL_TYPE_FLOAT || value.type == ESP_MATTER_VAL_TYPE_NULLABLE_FLOAT) {
        ESP_LOGI(TAG, "%s: %s. Value is %f", name, action_name, value.val.f);
    } else {
      val_print(endpoint_id, cluster_id, attribute_id, &value, true);
    }
}

void __matter_sensor_reporter(void* arg) {
    node_t *node = node::get();
    update_attribute(node, temperature_endpoint_id, TemperatureMeasurement::Id,
                     TemperatureMeasurement::Attributes::MeasuredValue::Id,
                     esp_matter_int16(__g_sensor_values.temperature.load()), "Temperature");
    update_attribute(node, humidity_endpoint_id, RelativeHumidityMeasurement::Id,
                     RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                     esp_matter_int16(__g_sensor_values.humidity.load()), "Humidity");
}

void __button2_callback(bool state) {
    uint16_t endpoint_id = extended_color_light_endpoint2_id;
    uint32_t cluster_id = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, cluster_id);
    attribute_t *attribute = attribute::get(cluster, attribute_id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.b = state;

    ESP_LOGI(TAG, "Dimmer on/off: esp_matter_attr_val_t value is %d", (int)val.val.b);
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void __button1_callback(bool state) {
    uint16_t endpoint_id = extended_color_light_endpoint_id;
    uint32_t cluster_id = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, cluster_id);
    attribute_t *attribute = attribute::get(cluster, attribute_id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.b = state;

    ESP_LOGI(TAG, "Dimmer switch: esp_matter_attr_val_t value is %d", (int)val.val.b);
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void __view_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    switch (id)
    {
        case VIEW_EVENT_SHUTDOWN: {
            ESP_LOGI(TAG, "event: VIEW_EVENT_SHUTDOWN");
            break;
        }
        case VIEW_EVENT_MATTER_DASHBOARD_DATA: {
            ESP_LOGI(TAG, "event: VIEW_EVENT_MATTER_DASHBOARD_DATA");
            if (!__g_matter_connected_flag.load()) {
                return;
            }
            struct view_data_matter_dashboard_data  *p_data = (struct view_data_matter_dashboard_data *) event_data;

            switch (p_data->dashboard_data_type)
            {
                case DASHBOARD_DATA_ARC: {
                    uint16_t endpoint_id = extended_color_light_endpoint_id;
                    uint32_t cluster_id = LevelControl::Id;
                    uint32_t attribute_id = LevelControl::Attributes::CurrentLevel::Id;

                    node_t *node = node::get();
                    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
                    cluster_t *cluster = cluster::get(endpoint, cluster_id);
                    attribute_t *attribute = attribute::get(cluster, attribute_id);

                    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
                    attribute::get_val(attribute, &val);
                    val.val.u8 = REMAP_TO_RANGE(p_data->value, STANDARD_BRIGHTNESS, MATTER_BRIGHTNESS);
                    if (p_data->value > 0) {
                        __button1_callback(true);
                    } else {
                        __button1_callback(false);
                    }
                    
                    ESP_LOGI(TAG, "Dimmer switch: esp_matter_attr_val_t value is %d", val.val.i);
                    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
                    break;
                }
                case DASHBOARD_DATA_SWITCH: {
                    uint16_t endpoint_id = door_lock_endpoint_id;
                    uint32_t cluster_id = DoorLock::Id;
                    uint32_t attribute_id = DoorLock::Attributes::LockState::Id;
                    node_t *node = node::get();
                    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
                    cluster_t *cluster = cluster::get(endpoint, cluster_id);
                    attribute_t *attribute = attribute::get(cluster, attribute_id);

                    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
                    attribute::get_val(attribute, &val);
                    val.val.b = (bool)p_data->value;
                    ESP_LOGI(TAG, "Door lock: esp_matter_attr_val_t value is %d", (int)val.val.b);    
                    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
                    break;
                }
                case DASHBOARD_DATA_SLIDER: {
                    uint16_t endpoint_id = extended_color_light_endpoint2_id;
                    uint32_t cluster_id = LevelControl::Id;
                    uint32_t attribute_id = LevelControl::Attributes::CurrentLevel::Id;

                    node_t *node = node::get();
                    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
                    cluster_t *cluster = cluster::get(endpoint, cluster_id);
                    attribute_t *attribute = attribute::get(cluster, attribute_id);

                    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
                    attribute::get_val(attribute, &val);
                    val.val.u8 = REMAP_TO_RANGE(p_data->value, STANDARD_BRIGHTNESS, MATTER_BRIGHTNESS);
                    if (p_data->value > 0) {
                        __button2_callback(true);
                    } else {
                        __button2_callback(false);
                    }

                    ESP_LOGI(TAG, "Dimmer switch: esp_matter_attr_val_t value is %d", val.val.i);    
                    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
                    break;
                }
                case DASHBOARD_DATA_BUTTON1: {
                    __button1_callback((bool)p_data->value);
                    break;
                }
                case DASHBOARD_DATA_BUTTON2: {
                    __button2_callback((bool)p_data->value);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case VIEW_EVENT_SENSOR_DATA: {
            ESP_LOGI(TAG, "event: VIEW_EVENT_SENSOR_DATA");

            struct view_data_sensor_data  *p_data = (struct view_data_sensor_data *) event_data;

            switch (p_data->sensor_type)
            {
                case SENSOR_DATA_CO2: {
                    break;
                }
                case SENSOR_DATA_TVOC: {
                    break;
                }
                case SENSOR_DATA_TEMP: {
                    __g_sensor_values.temperature = (int16_t)(p_data->value*100);
                    break;
                }
                case SENSOR_DATA_HUMIDITY: {
                    __g_sensor_values.humidity = (int16_t)(p_data->value*100);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }
}

} // namespace

int indicator_matter_setup(void) {
    esp_err_t err = ESP_OK;
    __g_matter_connected_flag = chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    // Create the temperature endpoint
    temperature_sensor::config_t temperature_config;
    endpoint_t *temperature_endpoint = temperature_sensor::create(node, &temperature_config, ENDPOINT_FLAG_NONE, NULL);

    // Create the humidity endpoint
    humidity_sensor::config_t humidity_config;
    endpoint_t *humidity_endpoint = humidity_sensor::create(node, &humidity_config, ENDPOINT_FLAG_NONE, NULL);

    // Create the dimmable light endpoint
    extended_color_light::config_t extended_color_light_config;
    extended_color_light_config.on_off.on_off = false;
    extended_color_light_config.level_control.current_level = 1;
    extended_color_light_config.level_control.lighting.start_up_current_level = 1;
    endpoint_t *extended_color_light_endpoint = extended_color_light::create(node, &extended_color_light_config, ENDPOINT_FLAG_NONE, NULL);

    // Create the contact sensor endpoint
    door_lock::config_t door_lock_config;
    endpoint_t *door_lock_endpoint = door_lock::create(node, &door_lock_config, ENDPOINT_FLAG_NONE, NULL);

    extended_color_light::config_t extended_color_light_config2;
    extended_color_light_config2.on_off.on_off = false;
    extended_color_light_config2.level_control.current_level = 1;
    extended_color_light_config2.level_control.lighting.start_up_current_level = 1;
    endpoint_t *extended_color_light_endpoint2 = extended_color_light::create(node, &extended_color_light_config2, ENDPOINT_FLAG_NONE, NULL);

    if (!node || 
        !temperature_endpoint || 
        !humidity_endpoint ||
        !extended_color_light_endpoint ||
        !door_lock_endpoint ||
        !extended_color_light_endpoint2
    ) {
        ESP_LOGE(TAG, "Matter node creation failed");
    }

    temperature_endpoint_id = endpoint::get_id(temperature_endpoint);
    ESP_LOGI(TAG, "Temperature sensor created with endpoint_id %d", temperature_endpoint_id);

    humidity_endpoint_id = endpoint::get_id(humidity_endpoint);
    ESP_LOGI(TAG, "Humidity sensor created with endpoint_id %d", humidity_endpoint_id);

    extended_color_light_endpoint_id = endpoint::get_id(extended_color_light_endpoint);
    ESP_LOGI(TAG, "Dimmable light created with endpoint_id %d", extended_color_light_endpoint_id);

    door_lock_endpoint_id = endpoint::get_id(door_lock_endpoint);
    ESP_LOGI(TAG, "Door lock created with endpoint_id %d", door_lock_endpoint_id);

    extended_color_light_endpoint2_id = endpoint::get_id(extended_color_light_endpoint2);
    ESP_LOGI(TAG, "Dimmable plugin 2 unit created with endpoint_id %d", extended_color_light_endpoint2_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Matter start failed: %d", err);
    }

    return err;
} 

int indicator_matter_init(void) {
    if (!__g_matter_connected_flag.load()) {
        ESP_LOGI(TAG, "Beginning Matter Provisioning");
        uint8_t screen = SCREEN_MATTER_CONFIG;
        ESP_ERROR_CHECK(esp_event_post_to(view_event_handle, VIEW_EVENT_BASE, VIEW_EVENT_SCREEN_START, &screen, sizeof(screen), portMAX_DELAY));
    }
    const esp_timer_create_args_t sensor_timer_args = {
            .callback = &__matter_sensor_reporter,
            /* argument specified here will be passed to timer callback function */
            .arg = (void*) matter_sensor_timer_handle,
            .name = "matter sensor update"
    };
    ESP_ERROR_CHECK(esp_timer_create(&sensor_timer_args, &matter_sensor_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(matter_sensor_timer_handle, 1000000 * MATTER_UPDATE_INTERVAL_IN_SECONDS)); //30 seconds

    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(view_event_handle,
                                                        VIEW_EVENT_BASE, VIEW_EVENT_SENSOR_DATA,
                                                        __view_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(view_event_handle,
                                                        VIEW_EVENT_BASE, VIEW_EVENT_MATTER_DASHBOARD_DATA,
                                                        __view_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register_with(view_event_handle, 
                                                            VIEW_EVENT_BASE, VIEW_EVENT_SHUTDOWN, 
                                                            __view_event_handler, NULL, NULL));
    return 0;                                                        
}

