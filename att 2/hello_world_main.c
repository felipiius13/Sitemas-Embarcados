#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"

void app_main(void)
{
	int segundos = 0;
	
    while (1) {
		printf("Nome: Felipe, Matricula: 202211250015\n");
		printf("Quantidade de nucleos: %d\n", configNUM_CORES);
		
		esp_chip_info_t chip_info;
		esp_chip_info(&chip_info);
		printf("Versao do silicio: %d\n", chip_info.revision);
		
		printf("Segundos decorridos: %d\n", segundos);
		
        vTaskDelay(pdMS_TO_TICKS(2000));
		segundos += 2;
    }
}