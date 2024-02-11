#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define NUNAVUT_ASSERT(x) (assert(x))

#include "serard.h"
#include "uavcan/node/Health_1_0.h"
#include "uavcan/node/Mode_1_0.h"
#include "uavcan/node/Heartbeat_1_0.h"

static const int HEARTBEAT_PERIOD = 1U;

static void* serardAlloc(void* user_reference, size_t size) {
    (void) user_reference;
    return malloc(size);
}

static void serardFree(void* user_reference, size_t size, void* ptr) {
    (void) user_reference;
    (void) size;
    free(ptr);
}

static const struct SerardMemoryResource allocator = {
    .user_reference = NULL,
    .deallocate = &serardFree,
    .allocate = &serardAlloc,
};

static bool serardEmitter(void* const user_reference, uint8_t data_size, const uint8_t* data) {
    int fd = *((int*) user_reference);
    ssize_t out = write(fd, data, data_size);
    if (out == -1) {
        printf("unable to write to serial port: error %d (%s)\n", errno, strerror(errno));
        return false;
    } else if (out < data_size) {
        printf("write failed: requested %d bytes, %ld bytes written\n", data_size, out);
        return false;
    }

    return true;
}

static void processHeartbeat(const struct SerardRxTransfer* const transfer) {
    const struct SerardTransferMetadata* const metadata = &transfer->metadata;
    assert(metadata->port_id == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);
    assert(transfer->payload != NULL);

    uavcan_node_Heartbeat_1_0 heartbeat;
    size_t inout_payload_size = transfer->payload_size;
    const int8_t ret = uavcan_node_Heartbeat_1_0_deserialize_(&heartbeat, transfer->payload, &inout_payload_size);
    serardFree(NULL, transfer->payload_extent, transfer->payload);
    if (ret < 0) {
        printf("failed to deserialize heartbeat\n");
        return;
    }

    const char* health;
    switch (heartbeat.health.value) {
        case uavcan_node_Health_1_0_NOMINAL:
            health = "caution";
            break;
        case uavcan_node_Health_1_0_ADVISORY:
            health = "advisory";
            break;
        case uavcan_node_Health_1_0_CAUTION:
            health = "caution";
            break;
        case uavcan_node_Health_1_0_WARNING:
            health = "warning";
            break;
    }

    const char* mode;
    switch (heartbeat.mode.value) {
        case uavcan_node_Mode_1_0_OPERATIONAL:
            mode = "operational";
            break;
        case uavcan_node_Mode_1_0_INITIALIZATION:
            mode = "initialization";
            break;
        case uavcan_node_Mode_1_0_MAINTENANCE:
            mode = "maintenance";
            break;
        case uavcan_node_Mode_1_0_SOFTWARE_UPDATE:
            mode = "software update";
            break;
        default:
            mode = "unknown";
            break;
    }

    printf("Heartbeat:\n"
           "Node ID: %d\n"
           "Health: %s\n"
           "Mode: %s\n"
           "Uptime: %d\n\n", metadata->remote_node_id, health, mode, heartbeat.uptime);
}

int main(int argc, char **argv) {
    if (argc != 4) goto print_usage;

    // open
    char *port = argv[1];
    int fd = open(port, O_RDWR);
    if (fd < 0) {
        printf("unable to open serial port %s: error %d (%s)\n", port, errno, strerror(errno));
        exit(1);
    }

    // configure
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        printf("unable to read port attributes: error %d (%s)\n", errno, strerror(errno));
        close(fd);
        exit(1);
    }

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // disable flow control
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // enable read, disable carrier detect
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cflag &= ~ISIG;
    // disable canonical mode and echo
    tty.c_cflag &= ~ICANON;
    tty.c_cflag &= ~(ECHO | ECHOE | ECHONL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    // read rate
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    // baud
    int baud = atoi(argv[2]);
    if (baud == 0) {
        printf("invalid baud rate: %s\n", argv[2]);
        close(fd);
        goto print_usage;
    }

    // only common speeds enumerated out of laziness
    speed_t speed;
    switch (baud) {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            printf("invalid baud rate: %s\n", argv[2]);
            close(fd);
            goto print_usage;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("unable to write port attributes: error %d (%s)\n", errno, strerror(errno));
        close(fd);
        exit(1);
    }

    SerardNodeID node_id = atoi(argv[3]);
    if (node_id == 0) {
        node_id = SERARD_NODE_ID_UNSET;
    }

    struct Serard serard = serardInit(allocator, allocator);
    serard.node_id = node_id;

    struct SerardRxSubscription heartbeat_subscription;
    int8_t ret = serardRxSubscribe(&serard, SerardTransferKindMessage, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, uavcan_node_Heartbeat_1_0_EXTENT_BYTES_, 1000000, &heartbeat_subscription);
    if (ret < 0) {
        printf("unable to subscribe to heartbeat message: error %d\n", ret);
        close(fd);
        exit(1);
    }

    struct SerardReassembler reassembler = serardReassemblerInit();

    clock_t last_heartbeat = 0;
    int32_t out;
    uint64_t transfer_id = 0;
    while (1) {
        const clock_t uptime = clock();
        if (((uptime - last_heartbeat) / CLOCKS_PER_SEC) > HEARTBEAT_PERIOD) {
            last_heartbeat = uptime;

            const uavcan_node_Health_1_0 health = {
                .value = uavcan_node_Health_1_0_NOMINAL,
            };
            const uavcan_node_Mode_1_0 mode = {
                .value = uavcan_node_Mode_1_0_OPERATIONAL,
            };
            const uavcan_node_Heartbeat_1_0 heartbeat = {
                .uptime = uptime / CLOCKS_PER_SEC,
                .health = health,
                .mode = mode,
                .vendor_specific_status_code = 0,
            };

            uint8_t buffer[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
            size_t buffer_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
            out = uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, buffer, &buffer_size);
            if (out != 0) {
                printf("failed to serialize heartbeat message\n");
                close(fd);
                exit(1);
            }

            const struct SerardTransferMetadata metadata = {
                .priority = SerardPriorityNominal,
                .transfer_kind = SerardTransferKindMessage,
                .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                .remote_node_id = SERARD_NODE_ID_UNSET,
                .transfer_id = transfer_id++,
            };
            out = serardTxPush(&serard, &metadata, buffer_size, buffer, (void*) &fd, &serardEmitter);
            if (out < 0) {
                printf("failed to send heartbeat message (invalid argument)\n");
                close(fd);
                exit(1);
            } else if (out == 0) {
                printf("failed to send heartbeat message (emitter failure)\n");
                close(fd);
                exit(1);
            }
        }

        uint8_t buf[256];
        const int bytes_read = read(fd, buf, sizeof(buf));
        if (bytes_read < 0) {
            printf("failed to read from serial port: error %d (%s)\n", errno, strerror(errno));
            close(fd);
            exit(1);
        } else if (bytes_read > 0) {
            size_t inout_payload_size = bytes_read;
            struct SerardRxTransfer transfer;
            struct SerardRxSubscription *sub;
            uint8_t *ptr = buf;
            int8_t ret;
            while (true) {
                ret = serardRxAccept(&serard, &reassembler, uptime, &inout_payload_size, ptr, &transfer, &sub);
                if (ret < 0) {
                    printf("failed to accept rx bytes: error %d\n", ret);
                    close(fd);
                    exit(1);
                } else if (ret > 0) {
                    // received a transfer
                    processHeartbeat(&transfer);
                }

                if (inout_payload_size > 0) {
                    ptr += inout_payload_size;
                } else {
                    break;
                }
            }
        }
    }

    return 0;

print_usage:
    printf("Usage:\n"
           "heartbeat <port> <baud> <node id>\n");
    exit(1);
}
