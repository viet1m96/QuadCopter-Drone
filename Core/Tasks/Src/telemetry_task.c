#include "telemetry_task.h"
#include "stdio.h"
#include "sdcard.h"
#include "string.h"
#define TELEMETRY_TASK_STACK_DEPTH 512U
#define TELEMETRY_PRIORITY (tskIDLE_PRIORITY + 1U)

#define TELEMETRY_SECTOR_MAGIC        0x474F4C54U
#define TELEMETRY_SECTOR_VERSION      1U
#define TELEMETRY_SAMPLES_PER_SECTOR  6U
#define TELEMETRY_FIRST_SECTOR        0U




typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t sample_cnt;
	uint32_t session_id;
	uint32_t sequence;
	uint32_t sample_size;
	uint8_t reserved[12];
} TelemetrySectorHeader_t;



typedef struct {
	TelemetrySectorHeader_t sector_header;
	TelemetrySample_t sample[TELEMETRY_SAMPLES_PER_SECTOR];
} TelemetrySector_t;

static uint32_t tele_session_id;
static TelemetrySector_t tele_sector;
static uint32_t tele_next_sector;
static uint32_t tele_sector_sequence;
static uint8_t tele_logging_enabled;

static void TelemetryLog_BeginSector(void) {
	memset(&tele_sector, 0, sizeof(tele_sector));
	tele_sector.sector_header.magic = TELEMETRY_SECTOR_MAGIC;
	tele_sector.sector_header.version = TELEMETRY_SECTOR_VERSION;
	tele_sector.sector_header.sample_cnt = 0U;
	tele_sector.sector_header.session_id = tele_session_id;
	tele_sector.sector_header.sequence = tele_sector_sequence;
	tele_sector.sector_header.sample_size = sizeof(TelemetrySample_t);
}


static SD_Status_t TelemetryLog_Start(void) {

	SD_Status_t status = SD_ReadSector(TELEMETRY_FIRST_SECTOR, (uint8_t*)&tele_sector);

	if(status != SD_OK) {
		return status;
	}
	uint32_t previous_session_id = 0U;
	if(tele_sector.sector_header.magic == TELEMETRY_SECTOR_MAGIC &&
	   tele_sector.sector_header.version == TELEMETRY_SECTOR_VERSION) {
		previous_session_id = tele_sector.sector_header.session_id;
	}
	tele_session_id = previous_session_id + 1U;
	tele_next_sector = TELEMETRY_FIRST_SECTOR;
	tele_sector_sequence = 0U;
	tele_logging_enabled = 1U;
	TelemetryLog_BeginSector();
	return SD_OK;
}

static SD_Status_t TelemetryLog_FlushSector(void) {
	SD_Status_t status;
	if(tele_sector.sector_header.sample_cnt == 0U) {
		return SD_OK;
	}
	status = SD_WriteSector(tele_next_sector, (const uint8_t*)&tele_sector);
	if(status != SD_OK) return status;
	tele_next_sector++;
	tele_sector_sequence++;
	TelemetryLog_BeginSector();
	return SD_OK;
}

static SD_Status_t TelemetryLog_AppendSample(const TelemetrySample_t* sample) {
	if(sample == NULL) return SD_ERR_NULL;
	if(tele_logging_enabled == 0U) return SD_OK;
	tele_sector.sample[tele_sector.sector_header.sample_cnt] = *sample;
	tele_sector.sector_header.sample_cnt++;
	if(tele_sector.sector_header.sample_cnt == TELEMETRY_SAMPLES_PER_SECTOR) {
		return TelemetryLog_FlushSector();
	}
	return SD_OK;
}

static void TelemetryTask(void *argument) {
  TelemetryTask_Context_t *ctx = (TelemetryTask_Context_t *)argument;
  TelemetrySample_t sample;
  SD_Status_t sd_status = SD_Init();
  if(sd_status == SD_OK) {
	  sd_status = TelemetryLog_Start();
  }
  if(sd_status != SD_OK) {
	  tele_logging_enabled = 0U;
  }
  for (;;) {
	  if(xQueueReceive(ctx->sample_queue, &sample, portMAX_DELAY) != pdPASS) {
		  continue;
	  }
	  if(tele_logging_enabled == 0U) continue;
	  sd_status = TelemetryLog_AppendSample(&sample);
	  if(sd_status != SD_OK) {
		  tele_logging_enabled = 0U;
	  }
  }
}

BaseType_t TelemetryTask_Create(TelemetryTask_Context_t *tele_context) {

  if (tele_context == NULL  ||
      tele_context->sample_queue == NULL)
    return pdFAIL;
  return xTaskCreate(TelemetryTask, "TelemetryTask", TELEMETRY_TASK_STACK_DEPTH,
                     tele_context, TELEMETRY_PRIORITY, NULL);
}
