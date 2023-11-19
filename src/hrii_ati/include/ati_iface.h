#ifndef __ATI_IFACE_H__
#define __ATI_IFACE_H__

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <boost/circular_buffer.hpp>

#include <mutex>
#include <string>

#define LOG_SIZE   10000000 

//#include <iit/ecat/utils.h>


typedef struct {
    uint16_t hdr;
    uint16_t cmd;
    uint32_t n_samples;
} cmd_t;

typedef struct {
    uint32_t    rtd_seq;
    uint32_t    ft_seq;
    uint32_t    status;
    int32_t     ft[6];
} ati_raw_t; // 36 bytes

typedef struct {
    uint32_t    rtd_seq;
    uint64_t    ts;
    float       ft[6];
    void sprint(char *buff, size_t size) {
        snprintf(buff, size, "%lu\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", ts, rtd_seq, ft[0],ft[1],ft[2],ft[3],ft[4],ft[5]);
    }
    void fprint(FILE *fp) {
        fprintf(fp, "%lu\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", ts, rtd_seq, ft[0],ft[1],ft[2],ft[3],ft[4],ft[5]);
    }
} ati_log_t ; // 36 bytes




class Ati_Sens  
{


private:
    bool run;
    pthread_t   thread_id;

    std::mutex  mtx;
    ati_log_t   last_sample;

    int recv_data();
    int send_cmd(cmd_t &);

public:

    Ati_Sens(const char* atiIp, bool run = false);
    virtual ~Ati_Sens(void);

    void get_last_sample(ati_log_t &sample);

protected:

    void start_thread(void);
    static void * rx_thread(void *);

    int udp_sock;
    //struct sockaddr_in local_addr;
    struct sockaddr_in  dest_addr;

    uint64_t    start_time;
    boost::circular_buffer<ati_log_t> ati_log;

};



#endif
