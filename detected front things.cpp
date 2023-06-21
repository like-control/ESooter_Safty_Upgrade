// ALSA 핸들 초기화
snd_pcm_t *pcm_handle;
if (snd_pcm_open(&pcm_handle, device, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
    printf("오디오 장치를 열 수 없습니다.\n");
    return -1;
}

// WAV 파일 열기
FILE *wav_file = fopen(file, "rb");
if (!wav_file) {
    printf("WAV 파일을 열 수 없습니다.\n");
    return -1;
}

// WAV 헤더 읽기
char header[44];
if (fread(header, sizeof(header), 1, wav_file) != 1) {
    printf("WAV 헤더를 읽을 수 없습니다.\n");
    return -1;
}

// PCM 파라미터 설정
unsigned int sample_rate = *(unsigned int*)&header[24];
unsigned int channels = *(unsigned short*)&header[22];
snd_pcm_set_params(pcm_handle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                channels, sample_rate, 1, 500000);

// 소리 데이터 재생
const int buffer_size = 1024;
char buffer[buffer_size];
int read_size;
while ((read_size = fread(buffer, 1, buffer_size, wav_file)) > 0) {
    snd_pcm_writei(pcm_handle, buffer, read_size / 4); // 2바이트(16비트) 샘플이므로 나누기 4
}

// 리소스 정리
snd_pcm_drain(pcm_handle);
snd_pcm_close(pcm_handle);
fclose(wav_file);