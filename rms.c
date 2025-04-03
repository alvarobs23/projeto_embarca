#include <stdio.h> 
#include <math.h> // biblioteca pra funções matematicas
#include "pico/stdlib.h" 
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include <pico-ssd1306/ssd1306.h> // biblioteca pra  o display oled
#include <string.h>

// Definições de pinos sensor ADXL345 e I2C
#define ADXL345_SDA_PIN 8
#define ADXL345_SCL_PIN 9
#define ADXL345_I2C i2c0

// Definições de pinos sensor Display oled e I2C
#define OLED_SDA_PIN 14
#define OLED_SCL_PIN 15
#define OLED_I2C i2c1

// Endereço I2C do ADXL345
#define ADXL345_ADDR 0x53

// Endereço dos registradores internos do ADXL345
#define ADXL345_REG_DEVID         0x00
#define ADXL345_REG_POWER_CTL     0x2D // Controle de energia
#define ADXL345_REG_DATA_FORMAT   0x31 // Resolução
#define ADXL345_REG_BW_RATE       0x2C
#define ADXL345_REG_DATAX0        0x32

// Pinos do joystick
#define JOYSTICK_X_PIN 26
#define JOYSTICK_Y_PIN 27
#define JOYSTICK_BTN_PIN 5

// Taxa de amostragem (pontos por segundo)
#define SAMPLE_RATE 1600
#define SAMPLE_DT (1.0f / SAMPLE_RATE)

// Fatores de conversão e integração
#define G_TO_MM_S2 9806.65f // converte (g) para mm/s^2
#define CONVERSION_FACTOR (1.0f / SAMPLE_RATE)

// Número de amostras para cálculo do RMS (1 segundo de dados)
#define NUM_SAMPLES SAMPLE_RATE

// Offsets medidos para compensação (em g)
#define OFFSET_X  0.0033f
#define OFFSET_Y -0.0070f
#define OFFSET_Z -1.0178f

// Variáveis globais para navegação e escolhas
volatile uint8_t current_menu = 0; // 0: Menu principal, 1: Seleção de base, 2: Tela de diagnóstico
volatile uint8_t menu_selection = 0; // opção selecionada
uint32_t last_joystick_time = 0;
const uint32_t debounce_delay = 200; // 200 ms

// Escolhas do usuário
int motor_power_class = 0; // 0: Ate 20 CV, 1: 20 ate 400 CV, 2: Mais de 400 CV
int base_type = 0;         // 0: Flexível, 1: Rígida

// Variável para armazenar o valor RMS total de velocidade
volatile float rms_velocity = 0.0f;

// Buffer para leitura do sensor
uint8_t sensor_data[6];

// Instância do display OLED
ssd1306_t oled_display;
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Protótipos
const char *diagnostico_rms(void);

// Inicializa o ADXL345
bool adxl345_init() {
    uint8_t data;
    if (i2c_read_blocking(ADXL345_I2C, ADXL345_ADDR, &data, 1, false) != 1) {
        printf("Falha ao ler o device ID do ADXL345\n");
        return false;
    }
    printf("Device ID ADXL345: 0x%02x\n", data);

    data = 0x08; // Ativa medição
    if (i2c_write_blocking(ADXL345_I2C, ADXL345_ADDR, (uint8_t[]){ADXL345_REG_POWER_CTL, data}, 2, false) != 2) {
        printf("Falha ao configurar POWER_CTL\n");
        return false;
    }
    data = 0x08; // Full resolution, ±2g
    if (i2c_write_blocking(ADXL345_I2C, ADXL345_ADDR, (uint8_t[]){ADXL345_REG_DATA_FORMAT, data}, 2, false) != 2) {
        printf("Falha ao configurar DATA_FORMAT\n");
        return false;
    }
    data = 0x0F; // Taxa de dados (ex.: 1600Hz)
    if (i2c_write_blocking(ADXL345_I2C, ADXL345_ADDR, (uint8_t[]){ADXL345_REG_BW_RATE, data}, 2, false) != 2) {
        printf("Falha ao configurar BW_RATE\n");
        return false;
    }
    printf("ADXL345 inicializado com sucesso.\n");
    return true;
}

// Lê acelerações (X, Y, Z) em "g" e aplica compensação de offset
void adxl345_read(float *ax, float *ay, float *az) {
    if (i2c_write_blocking(ADXL345_I2C, ADXL345_ADDR, (uint8_t[]){ADXL345_REG_DATAX0}, 1, true) != 1) {
        printf("Erro ao solicitar leitura dos dados do sensor.\n");
        return;
    }
    if (i2c_read_blocking(ADXL345_I2C, ADXL345_ADDR, sensor_data, 6, false) != 6) {
        printf("Erro na leitura dos 6 bytes de dados.\n");
        return;
    }
    int16_t raw_x = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
    int16_t raw_y = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
    int16_t raw_z = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);

    // Converte para "g" e compensa os offsets medidos
    *ax = raw_x * 0.0039f - OFFSET_X;
    *ay = raw_y * 0.0039f - OFFSET_Y;
    *az = raw_z * 0.0039f - OFFSET_Z;
}

// Lê o ADC do canal especificado
uint16_t read_joystick_adc(uint8_t channel) {
    adc_select_input(channel);
    return adc_read();
}


// Diagnóstico baseado nos valores RMS com dois limites fixos (normal e alerta)
const char *diagnostico_rms(void) {
    // thresholds[motor_power_class][base_type][0] = limite normal, [1] = limite de alerta
    float thresholds[3][2][2] = {
        {   // Motor até 20 CV
            {3.0f, 4.0f},    // Base Flexível
            {2.8f, 4.5f}     // Base Rígida
        },
        {   // Motor de 20 até 400 CV
            {4.5f, 6.5f},    // Base Flexível
            {4.0f, 6.0f}     // Base Rígida
        },
        {   // Motor acima de 400 CV
            {6.5f, 8.5f},    // Base Flexível
            {6.0f, 8.0f}     // Base Rígida
        }
    };

    float normal_limit = thresholds[motor_power_class][base_type][0]; // armazena o limite escolhido
    float alert_limit  = thresholds[motor_power_class][base_type][1]; // armazena o limite escolhido

    if (rms_velocity <= normal_limit) { // compara a rms_velocity com o limite
        return "Condicao: Normal";
    } else if (rms_velocity <= alert_limit) {
        return "Condicao: Alerta";
    } else {
        return "Condicao: Emergencial";
    }
}


// Atualiza o display OLED de acordo com o menu atual
void update_oled_menu() { // atualiza o display
    ssd1306_clear(&oled_display); // limpa o display
    char buffer[32];

    if (current_menu == 0) {
        ssd1306_draw_string(&oled_display, 0, 0, 1, "Classif. por Potencia");
        sprintf(buffer, "1) Ate 20 CV%s", (menu_selection==0)?" <-": "");
        ssd1306_draw_string(&oled_display, 0, 12, 1, buffer);
        sprintf(buffer, "2) 20 a 400 CV%s", (menu_selection==1)?" <-": "");
        ssd1306_draw_string(&oled_display, 0, 24, 1, buffer);
        sprintf(buffer, "3) Mais de 400 CV%s", (menu_selection==2)?" <-": "");
        ssd1306_draw_string(&oled_display, 0, 36, 1, buffer);
    } else if (current_menu == 1) {
        ssd1306_draw_string(&oled_display, 0, 0, 1, "Tipo de Base:");
        sprintf(buffer, "1) Flexivel%s", (menu_selection==0)?" <-": "");
        ssd1306_draw_string(&oled_display, 0, 12, 1, buffer);
        sprintf(buffer, "2) Rigida%s", (menu_selection==1)?" <-": "");
        ssd1306_draw_string(&oled_display, 0, 24, 1, buffer);
    } else if (current_menu == 2) {
        sprintf(buffer, "RMS Total: %.3f mm/s", rms_velocity);
        ssd1306_draw_string(&oled_display, 0, 0, 1, buffer);
        const char *diag = diagnostico_rms();
        ssd1306_draw_string(&oled_display, 0, 12, 1, diag);
        ssd1306_draw_string(&oled_display, 0, 36, 1, "Aperte Botao p/ Menu");
    }
    ssd1306_show(&oled_display); //atualiza o display
}

// Processa os dados do sensor e calcula o RMS dos três eixos
void process_sensor_data() {
    static float vel_samples_x[NUM_SAMPLES];
    static float vel_samples_y[NUM_SAMPLES];
    static float vel_samples_z[NUM_SAMPLES];
    static int sample_index = 0;
    
    float ax, ay, az;
    adxl345_read(&ax, &ay, &az);
    
    // conversao da aceleração pra velocidade (mm/s)
    float vel_sample_x = ax * G_TO_MM_S2 * CONVERSION_FACTOR;
    float vel_sample_y = ay * G_TO_MM_S2 * CONVERSION_FACTOR;
    float vel_sample_z = az * G_TO_MM_S2 * CONVERSION_FACTOR;
    
    vel_samples_x[sample_index] = vel_sample_x;
    vel_samples_y[sample_index] = vel_sample_y;
    vel_samples_z[sample_index] = vel_sample_z;
    
    sample_index++;
    
    if (sample_index >= NUM_SAMPLES) { // verifica se o numero de amostras atingiu o limite
        float sum_sq_x = 0.0f, sum_sq_y = 0.0f, sum_sq_z = 0.0f;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            sum_sq_x += vel_samples_x[i] * vel_samples_x[i];
            sum_sq_y += vel_samples_y[i] * vel_samples_y[i];
            sum_sq_z += vel_samples_z[i] * vel_samples_z[i];
        }
        float rms_total = sqrtf((sum_sq_x + sum_sq_y + sum_sq_z) / NUM_SAMPLES);
        rms_velocity = rms_total;
        
        printf("Acel (compensada): X=%.3f g, Y=%.3f g, Z=%.3f g | RMS Total=%.3f mm/s\n", ax, ay, az, rms_total);
        
        sample_index = 0;
        if (current_menu == 2) {
            update_oled_menu();
        }
    }
}

// Inicializa o display OLED
bool oled_init() {
    i2c_init(OLED_I2C, 400 * 1000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);

    if (!ssd1306_init(&oled_display, OLED_WIDTH, OLED_HEIGHT, 0x3C, OLED_I2C)) {
        printf("Falha na inicializacao do OLED.\n");
        return false;
    }
    printf("OLED inicializado com sucesso.\n");
    return true;
}

// Atualiza a navegação dos menus
void update_menu_navigation() {
    uint32_t now = to_ms_since_boot(get_absolute_time()); // Converte o tempo em milissigundos
    if (now - last_joystick_time < debounce_delay) return; // Verifica o tempo da ultima interação com o joystick

    // Usa o ADC do eixo Y para navegação vertical 
    uint16_t adc_val = read_joystick_adc(0); // Variavel armazena o valor lido do ADC
    uint8_t max_option = (current_menu == 0) ? 2 : (current_menu == 1) ? 1 : 0; // Define o numero maximo de opções por menu

    // Ajusta os limiares conforme a resposta do ADC
    if (adc_val > 3000 && menu_selection > 0) { // Verifica se o valor lido do ADC é maior que 3000
        menu_selection--; // Decrementa a variável, move a seleção para a opção anterior.
        last_joystick_time = now; //Atualiza a variável last_joystick_time para o tempo atual
        update_oled_menu(); //atualiza o display OLED 
    } else if (adc_val < 1000 && menu_selection < max_option) { // verifica se o valor do ADC é menor que 1000, Joystick na direção oposta
        menu_selection++; //Incrementa a variável, movendo para a próxima opção do menu.
        last_joystick_time = now; //Atualiza o tempo da última ação do joystick para o tempo atual.
        update_oled_menu(); //Atualiza o display OLED 
    }
}

// Verifica o botão do joystick com debounce e espera pelo release
bool check_joystick_button() {
    static uint32_t last_btn_time = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());// representa o instante atual em milissegundos.
    if (now - last_btn_time < debounce_delay) return false; // Verifica se o intervalo de tempo entre o momento atual  e o último momento em que o botão foi processado é menor que o valor definido em debounce_delay.

    if (gpio_get(JOYSTICK_BTN_PIN) == 0) { // Lê o estado do botão
        while (gpio_get(JOYSTICK_BTN_PIN) == 0) { // Espera o botão retornar 1 para considerar a ação
            tight_loop_contents(); //função de espera 
        }
        last_btn_time = now;// após o botão ser solta ele atualiza com tempo atual
        return true;
    }
    return false;
}

int main() {
    stdio_init_all();
    printf("Iniciando o sistema...\n");

    // Inicializa I2C para o ADXL345
    i2c_init(ADXL345_I2C, 400 * 1000);
    gpio_set_function(ADXL345_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ADXL345_SCL_PIN, GPIO_FUNC_I2C);

    // Inicializa ADC para o joystick
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Configura o pino do botão do joystick com pull-up
    gpio_init(JOYSTICK_BTN_PIN);
    gpio_set_dir(JOYSTICK_BTN_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_BTN_PIN);

    // Inicializa o display OLED
    if (!oled_init()) {
        printf("Erro na inicializacao do OLED. Abortando...\n");
        return 1;
    }

    // Inicializa o ADXL345
    if (!adxl345_init()) {
        printf("Erro na inicializacao do sensor ADXL345. Abortando...\n");
        return 1;
    }

    // Exibe o menu inicial
    update_oled_menu();

    absolute_time_t next_sample_time = make_timeout_time_us(625);

    while (true) { // loop principal 
        if (current_menu == 0 || current_menu == 1) { // se estiver no menu 0 ou 1
            update_menu_navigation(); //atualiza o oled conforme o joystick é movimentado
            if (check_joystick_button()) { // se o botão for precionado
                printf("Opcao selecionada: %d no menu %d\n", menu_selection, current_menu);
                if (current_menu == 0) { // armazena a escolha do usuario
                    motor_power_class = menu_selection;
                } else if (current_menu == 1) { // armazena a escolha do usuario
                    base_type = menu_selection;
                }
                current_menu++; // Após selecionar ,passa para o proximo menu
                menu_selection = 0;
                update_oled_menu();
            }
        }
        if (current_menu == 2) { //começa a coletar e processar dados do sensor.
            process_sensor_data();
            if (check_joystick_button()) { // se o botão for selecionado ele volta pra o menu 0
                current_menu = 0;
                menu_selection = 0;
                update_oled_menu();
            }
        }
        busy_wait_until(next_sample_time);
        next_sample_time = delayed_by_us(next_sample_time, 625);
    }

    return 0;
}