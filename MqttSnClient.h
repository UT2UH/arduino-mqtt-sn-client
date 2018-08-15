
#ifndef ARDUINO_MQTTSN_CLIENT_MQTTSNCLIENT_H
#define ARDUINO_MQTTSN_CLIENT_MQTTSNCLIENT_H

#include "MqttSnMessageHandler.h"
#include "System.h"
#include <cstdint>
#include "global_defines.h"
#include "mqttsn_messages.h"

template<class MqttSnMessageHandler_SocketInterface>
class MqttSnMessageHandler;


struct topic_registration {
    char *topic_name;
    uint16_t topic_id;
    uint8_t granted_qos;
};

#ifdef ESP8266
#include <functional>
#define MQTT_SN_CALLBACK_SIGNATURE std::function<void(char*, uint8_t*, uint16_t, bool retain)> callback
#else
#define MQTT_SN_CALLBACK_SIGNATURE void (*callback)(char*, uint8_t*, uint16_t, bool retain)
#endif


template<class MqttSnClient_SocketInterface>
class MqttSnClient {
private:
    MqttSnClient_SocketInterface &socketInterface;
    MqttSnMessageHandler<MqttSnClient_SocketInterface> mqttSnMessageHandler;

    template<typename MqttSnMessageHandler_SocketInterface> friend
    class PacketHandler;

    System system;

    device_address gw_address;
    bool socket_disconnected = false;
    bool mqttsn_connected = false;
    message_type await_msg_type = MQTTSN_GWINFO;
    bool ping_outstanding = false;

    MQTT_SN_CALLBACK_SIGNATURE;

    uint16_t msg_id_counter = 1;
    bool await_topic_id = false;

    //we supports only single subscription at the moment
    topic_registration registration;
public:
    explicit MqttSnClient(MqttSnClient_SocketInterface &socketInterface) : socketInterface(
            socketInterface), mqttSnMessageHandler(socketInterface, *this) {}

    bool begin() {
        socketInterface.setMqttSnMessageHandler(&mqttSnMessageHandler);
        return mqttSnMessageHandler.begin();
    }


    bool is_gateway_address(device_address *pAddress) {
        return memcmp(&this->gw_address, pAddress, sizeof(device_address)) == 0;
    }

    void notify_socket_disconnected() {
        this->socket_disconnected = true;
    }

    void set_await_message(message_type msg_type) {
        this->await_msg_type = msg_type;
    }

    bool loop() {
        if (!mqttsn_connected) {
            return false;
        }
        if (socket_disconnected) {
            set_await_message(MQTTSN_PINGREQ);
            return false;
        }
        socketInterface.loop();
        if (system.has_beaten()) {
            ping_outstanding = true;
        }
        if (ping_outstanding && await_msg_type == MQTTSN_PINGREQ) {
            mqttSnMessageHandler.send_pingreq(&gw_address);
            set_await_message(MQTTSN_PINGRESP);
        }
        return true;
    }

    void setCallback(MQTT_SN_CALLBACK_SIGNATURE) {
        this->callback = callback;
    }

    bool connect(device_address *address, const char *client_id, uint16_t duration) {
        // global verwalten
        uint8_t retries = 5;
        memset(&this->gw_address, 0, sizeof(device_address));
        for (uint8_t tries = 0; tries < retries; tries++) {
            system.set_heartbeat(15 * 1000);
            mqttSnMessageHandler.send_connect(address, client_id, duration);
            this->set_await_message(MQTTSN_CONNACK);
            while (!mqttsn_connected) {
                socketInterface.loop();
                if (mqttsn_connected) {
                    memcpy(&this->gw_address, address, sizeof(device_address));
                    system.set_heartbeat(duration * 1000);
                    return true;
                }
                if (system.has_beaten()) {
                    // timeout
                    break;
                }
                if (socket_disconnected) {
                    return false;
                }
            }
            system.sleep((tries + 1) * 10 * 1000);
        }
        // timeout - take another gateway
        return false;
    }

    void subscribe(const char *topic, uint8_t qos) {
        while (this->await_msg_type != MQTTSN_PINGREQ) {
            // wait until we have no other messages in flight
            if (!socketInterface.loop()) {
                return;
            }
        }
        mqttSnMessageHandler.send_subscribe(&gw_address, topic, qos);
        this->set_await_message(MQTTSN_SUBACK);
        memset(&registration, 0, sizeof(topic_registration));
        //strcpy(registration.topic_name, topic);
        registration.topic_name = (char *) topic;
        registration.granted_qos = qos;
        await_topic_id = true;
    }

    void publish(const char *payload, const char *topic, int8_t qos) {
if(qos > 0){
        while (this->await_msg_type != MQTTSN_PINGREQ) {
            // wait until we have no other messages in flight
            if (!socketInterface.loop()) {
                return;
            }
        }
}
        uint16_t payload_length = strlen(payload);
        uint16_t msg_id = this->increment_and_get_msg_id_counter();
        if (qos == 0) {
            msg_id = 0;
        }
if( qos == 1){
	    this->await_msg_type != MQTTSN_PUBACK;
}
        uint16_t topic_id = registration.topic_id;
        mqttSnMessageHandler.send_publish(&gw_address, (uint8_t *) payload, payload_length, msg_id, topic_id, true,
                                           false, qos, false);
if(qos > 0){
        while (this->await_msg_type != MQTTSN_PINGREQ) {
            // wait until we have no other messages in flight
            if (!socketInterface.loop()) {
                return;
            }
        }
}
    }

    message_type get_await_message() {
        return this->await_msg_type;
    }

    void set_mqttsn_connected() {
        this->mqttsn_connected = true;
    }

    void notify_socket_connected() {
        this->socket_disconnected = false;
    }


    void notify_pingresponse_arrived() {
        system.set_heartbeat(system.get_heartbeat());
        ping_outstanding = false;
        //TODO mention: client observer his timout value and manages connections only by HIS pingreq
        // pingrequeest from the gateway to the client do NOT reset this timer
        // TODO enhancement: both pingrequests and pingreqsponse reset the timer OR better:
        // ALL answers reset the timer
    }

    uint16_t increment_and_get_msg_id_counter() {
        if (++msg_id_counter == 0) {
            msg_id_counter = 1;
        }
        return msg_id_counter;
    }

    uint16_t get_await_message_id() {
        return msg_id_counter;
    }

    void set_await_topic_id(uint16_t topic_id) {
        registration.topic_id = topic_id;
    }

    void set_granted_qos(int8_t granted_qos) {
        registration.granted_qos = (uint8_t) granted_qos;
    }

    bool is_mqttsn_connected() {
        return mqttsn_connected;
    }

    void handle_publish(device_address *address, uint8_t *data, uint16_t data_len, uint16_t topic_id, bool retain,
                        int8_t qos) {
        // call callback :)
        if (topic_id == registration.topic_id && qos == registration.granted_qos) {
            callback(registration.topic_name, data, data_len, retain);
        }
    }


};


#endif //ARDUINO_MQTTSN_CLIENT_MQTTSNCLIENT_H
