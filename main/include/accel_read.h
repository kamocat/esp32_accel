void accel_reader_task(void * pvParameters);

struct accel_t {
    int16_t x,y,z;
};
struct chunk_t {
    size_t len;
    struct accel_t * data;
};