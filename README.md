# ESP_MODBUS_MQTT_MOSQUITTO
Código para controle da planta do LAMP(Laboratório da UFRN) usando modbus, MQTT e o node red como um SCADA.
Como dispositivo controlador foi usado o ESP32, o mesmo faz a requisição dos parâmetros de controle via MODBUS ao gateway da YOKOGAWA, faz a comunicação com o sistema de aquisição implementado no Node-Red via MQTT, e executa o controle.

