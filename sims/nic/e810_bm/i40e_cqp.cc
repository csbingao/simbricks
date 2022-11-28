#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <cassert>
#include <iostream>
using namespace std;
#include "sims/nic/e810_bm/e810_base_wrapper.h"
#include "sims/nic/e810_bm/e810_bm.h"

#include <bits/stdc++.h>
namespace i40e {

control_queue_pair::control_queue_pair(i40e_bm &dev_, uint32_t &reg_high_,
                               uint32_t &reg_low_, uint32_t &reg_head_,
                               uint32_t &reg_tail_)
    : queue_base("cqp", reg_head_, reg_tail_, dev_),
      reg_high(reg_high_),
      reg_low(reg_low_) {
  desc_len = 64;
  ctxs_init();
}

queue_base::desc_ctx &control_queue_pair::desc_ctx_create() {
  return *new admin_desc_ctx(*this, dev);
}

void control_queue_pair::create_cqp() {
  if (reg_high == 0 && reg_low == 0){
    return;
  }
  uint64_t high = (uint64_t) reg_high;
  uint64_t low = (uint64_t) reg_low;
  base = (high << 32) | low;
  len = 64;
  std::cout << "cqp ctx base addr: " << std::hex << base << logger::endl;
  if (!enabled) {
    cqp_ctx_fetch *dma = new cqp_ctx_fetch(*this, 64, cqp_ctx);
    dma->dma_addr_ = base;
    dma->len_ = 64;
    dev.runner_->IssueDma(*dma);
    enabled = true;
    dev.regs.reg_PFPE_CQPTAIL = 0;
    if (dev.regs.reg_PFPE_CCQPLOW == 0){
          dev.regs.reg_PFPE_CCQPSTATUS = 1<<31;
        }
        else{
          dev.regs.reg_PFPE_CCQPSTATUS = 1;
    }
  }
}

void control_queue_pair::reg_updated() {
  if (enabled){
    trigger();
  }
  
}

control_queue_pair::cqe_fetch::cqe_fetch(control_queue_pair &cqp, size_t len, void *buffer) : buf_addr(buffer), cqp_(cqp) {
  data_ = new char[len];
  len_ = len;
}

control_queue_pair::cqp_ctx_fetch::cqp_ctx_fetch(control_queue_pair &cqp, size_t len, void *buffer) : buf_addr(buffer), cqp_(cqp) {
  data_ = new char[len];
  len_ = len;
}

control_queue_pair::cqe_fetch::~cqe_fetch() {
}

control_queue_pair::cqp_ctx_fetch::~cqp_ctx_fetch() {
}

void control_queue_pair::cqe_fetch::done() {
  desc_ctx &ctx = *cqp_.desc_ctxs[0];
  
  memcpy(ctx.desc, data_, len_);
  std::cout << "cqe buffer: "<< logger::endl;
  for (int i = 0; i < 8; i++)
  {
    std::cout << std::hex << (uint64_t)((uint64_t*)data_)[i];
  }
  for (int i = 0; i < 8; i++)
  {
    std::cout << std::hex << (uint64_t)((uint64_t*)buf_addr)[i];
  }
  ctx.process();
  
  delete this;
}

void control_queue_pair::cqp_ctx_fetch::done() {
  memcpy(buf_addr, data_, len_);
  std::cout << "cqp buffer: "<< logger::endl;
  for (int i = 0; i < 8; i++)
  {
    std::cout << std::hex << (uint64_t)((uint64_t*)data_)[i];
  }
  for (int i = 0; i < 8; i++)
  {
    std::cout << std::hex << (uint64_t)((uint64_t*)buf_addr)[i];
  }
  cqp_.ctx_fetched();
  delete this;
}

void control_queue_pair::ctx_fetched(){
  uint8_t *ctx_p = reinterpret_cast<uint8_t *>(cqp_ctx);
  u64 temp;
  get_64bit_val(cqp_ctx, 0, &temp);
  u32 sq_size = FIELD_GET(IRDMA_CQPHC_SQSIZE, temp);
  u64 sq_pa;
  get_64bit_val(cqp_ctx, 8, &sq_pa);
  std::cout << "sq size: "<< sq_size<<logger::endl;
  std::cout << "sq pa:" << logger::endl;
  std::cout << sq_pa << logger::endl;
  cqe_base = sq_pa;
  len = 64;
  get_64bit_val(cqp_ctx, 24, &host_cq_pa);
  // trigger();
}

void control_queue_pair::trigger() {
  trigger_fetch();
  // trigger_process();
  // trigger_writeback();
}

void control_queue_pair::trigger_fetch() {
  if (!enabled)
    return;
  uint32_t tail = dev.regs.reg_PFPE_CQPTAIL;
  cqe_fetch *dma = new cqe_fetch(*this, 64, cqe);
  dma->write_ = false;
  dma->dma_addr_ = cqe_base + (tail)*64;
  std::cout << "cqe addr: "<< dma->dma_addr_<<logger::endl;
  dma->len_ = 64;
  dma->pos = 0;
  dev.runner_->IssueDma(*dma);

}

void control_queue_pair::trigger_writeback() {
  if (!enabled)
    return;
}

void control_queue_pair::trigger_process() {
  if (!enabled)
    return;
  desc_ctx &ctx = *desc_ctxs[0];
  ctx.state = desc_ctx::DESC_PROCESSING;
  ctx.process();
}

control_queue_pair::admin_desc_ctx::admin_desc_ctx(control_queue_pair &queue_,
                                               i40e_bm &dev_)
    : i40e::queue_base::desc_ctx(queue_), aq(queue_), dev(dev_) {
  d = reinterpret_cast<__le64 *>(desc);
}

void control_queue_pair::admin_desc_ctx::data_written(uint64_t addr, size_t len) {
  processed();
}

void control_queue_pair::admin_desc_ctx::data_write(uint64_t addr, size_t data_len,
                                      const void *buf) {
  dma_data_wb *data_dma = new dma_data_wb(*this, data_len);
  data_dma->write_ = true;
  data_dma->dma_addr_ = addr;
  memcpy(data_dma->data_, buf, data_len);
  dev.runner_->IssueDma(*data_dma);
}

void control_queue_pair::admin_desc_ctx::desc_compl_prepare(uint16_t retval,
                                                        uint16_t extra_flags) {
}

void control_queue_pair::admin_desc_ctx::desc_complete(uint16_t retval,
                                                   uint16_t extra_flags) {
  desc_compl_prepare(retval, extra_flags);
  processed();
}

void control_queue_pair::admin_desc_ctx::desc_complete_indir(uint16_t retval,
                                                         const void *data,
                                                         size_t len,
                                                         u64 buf_addr,
                                                         uint16_t extra_flags,
                                                         bool ignore_datalen) {

  data_write(buf_addr, len, data);
}

void control_queue_pair::admin_desc_ctx::prepare() {

}

void control_queue_pair::admin_desc_ctx::process() {
  u64 temp;
  get_64bit_val((__le64*)desc, 24, &temp);
  __le16 opcode = FIELD_GET(IRDMA_CQPSQ_OPCODE, temp);
  std::cout << "opcode is: " << opcode << logger::endl;
  std::cout << "before process tail is" << dev.regs.reg_PFPE_CQPTAIL << logger::endl;
  if (opcode == IRDMA_CQP_OP_QUERY_FPM_VAL){
    u64 query_buf_addr;
    __le64 return_buffer[22];
    memset(return_buffer, 0, sizeof(__le64)*22);
    // 0-7 bytes
    u64 first_sd_index = 0;
    u64 temp = 0;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_FIRST_PE_SD_INDEX, first_sd_index);
    u64 max_pe_sds = 4096;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_MAX_PE_SDS, max_pe_sds);
    set_64bit_val(return_buffer, 0, temp);

    // 8-15 bytes
    temp = 0;
    u64 max_cnt = 0x00040000;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_MAX_QPS, max_cnt);
    temp = 1<<32 | temp; // GLHMC_PEQPOBJSZ
    set_64bit_val(return_buffer, 8, temp);

    // 8-15 bytes
    temp = 0;
    u64 cq_max_cnt = 0x00080000;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_MAX_CQS, cq_max_cnt);
    temp = 1<<32 | temp; 
    set_64bit_val(return_buffer, 16, temp);

    // 64 bytes
    u64 xf_block_size = 64;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_XFBLOCKSIZE, xf_block_size);
    set_64bit_val(return_buffer, 64, temp);

    // 72 bytes
    u64 hmc_iw_q1 = 0x00040000;
    set_64bit_val(return_buffer, 72, hmc_iw_q1);

    // 80 bytes
    u64 q1_block_size = 64;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_Q1BLOCKSIZE, q1_block_size);
    set_64bit_val(return_buffer, 80, temp);

    u32 pble_max_cnt = 0x10000000;
    set_64bit_val(return_buffer, 112, pble_max_cnt);

    // 120 bytes
    u64 max_ceqs = 768;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_MAX_CEQS, max_ceqs);
    set_64bit_val(return_buffer, 120, temp);

    // 136 bytes
    u64 rrf_block_size = 64;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_Q1BLOCKSIZE, rrf_block_size);
    set_64bit_val(return_buffer, 136, temp);
    
    // 168 bytes
    u64 ooiscf_block_size = 64;
    temp = 0;
    temp = FIELD_PREP(IRDMA_QUERY_FPM_OOISCFBLOCKSIZE, ooiscf_block_size);
    temp = temp | (u32) 1;
    set_64bit_val(return_buffer, 168, temp);
    get_64bit_val((__le64*)desc, 32, &query_buf_addr);
    std::cout << "query_buf_addr: "<< query_buf_addr << logger::endl;
    desc_complete_indir(0,return_buffer, 176, query_buf_addr);
  } else if (opcode == IRDMA_CQP_OP_SHMC_PAGES_ALLOCATED) {

  } else if (opcode == IRDMA_CQP_OP_CREATE_CQ) {
    uint64_t shadow_buf_addr;
    // Get return buffer addr
    get_64bit_val((__le64*)desc, 32, &shadow_buf_addr);
    temp = 0;
    get_64bit_val((__le64*)desc, 24, &temp);
    u32 cq_id = temp & 0x3ffff;
    std::cout << "cq id: " << cq_id << logger::endl;

    if (cq_id > 0){
      
    } else {
      cqp_base = shadow_buf_addr;
      cnt = 0;
    }
    
    __le64 return_buffer[4];
    u64 return_shadow_buf_addr = cqp_base + cnt*32;
    
    memset(return_buffer, 0, sizeof(__le64)*4);

    // 8 Bytes
     set_64bit_val(return_buffer, 8, dev.cqp.host_cq_pa);

    // 24 bytes
    temp = 0;
    // polarity
    u64 get_polarity;
    get_64bit_val((__le64*)desc, 24, &get_polarity);
    u8 polarity = (u8)FIELD_GET(IRDMA_CQPSQ_WQEVALID, get_polarity);
    u32 ceq_id = FIELD_GET(IRDMA_CQPSQ_CEQ_CEQID, get_polarity);
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    std::cout << "tail here: "<<tail<<logger::endl;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
                  FIELD_PREP(IRDMA_CQ_VALID, polarity);
    set_64bit_val(return_buffer, 24, temp);
    u32 new_tail = FIELD_GET(IRDMA_CQ_WQEIDX, temp);
    std::cout << "new tail here: "<<new_tail<<logger::endl;
    std::cout << " create cq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, return_buffer, 32, return_shadow_buf_addr);
    if (cq_id > 1) {
      dev.cem.qena_updated(ceq_id);
    }
    cnt++;
    
  } else if (opcode == IRDMA_CQP_OP_QUERY_RDMA_FEATURES) {
    u64 shadow_buf_addr;
    get_64bit_val((__le64*)desc, 32, &shadow_buf_addr);
    __le64 return_buffer[4];
    memset(return_buffer, 0, sizeof(__le64)*4);
    temp = 0;
    u16 feat_cnt = 26;
    temp = FIELD_PREP(IRDMA_FEATURE_CNT, feat_cnt);
    set_64bit_val(return_buffer, 0, temp);
    desc_complete_indir(0, return_buffer, 32, shadow_buf_addr);


    u64 return_shadow_buf_addr = cqp_base + cnt*32;
    __le64 cqe_return_buffer[4];
    
    // 8 bytes
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);

    // 24 bytes
    temp = 0;
    u64 get_polarity;
    get_64bit_val((__le64*)desc, 24, &get_polarity);
    u8 polarity = (u8)FIELD_GET(IRDMA_CQPSQ_WQEVALID, get_polarity);
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " query rdma return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);
    cnt++;
    

  } else if (opcode == IRDMA_CQP_OP_CREATE_CEQ) {
    u64 ceq_base;
    get_64bit_val((__le64*)desc, 32, &ceq_base);
    __le64 return_buffer[4];
    memset(return_buffer, 0, sizeof(__le64)*4);
    temp = 0;
    set_64bit_val(return_buffer, 0, temp);
    desc_complete_indir(0, return_buffer, 32, ceq_base);


    u64 return_shadow_buf_addr = cqp_base + 32*cnt;
    __le64 cqe_return_buffer[4];
    
    // 8 bytes
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);

    // 24 bytes
    temp = 0;
    u64 get_polarity;
    get_64bit_val((__le64*)desc, 24, &get_polarity);
    u8 polarity = (u8)FIELD_GET(IRDMA_CQPSQ_WQEVALID, get_polarity);
    u32 ceq_id = FIELD_GET(IRDMA_CQPSQ_CEQ_CEQID, get_polarity);
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " create ceq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);
    dev.cem.qena_updated(ceq_id);
    completion_event_queue &ceq = static_cast<completion_event_queue &>(*dev.cem.ceqs[ceq_id]);
    ceq.ceq_base = ceq_base;
    ceq.ceq_id = ceq_id;
    cnt++;
  } else if (opcode == IRDMA_CQP_OP_CREATE_AEQ) {
    u64 aeq_base;
    get_64bit_val((__le64*)desc, 32, &aeq_base);
    __le64 return_buffer[4];
    memset(return_buffer, 0, sizeof(__le64)*4);
    
    desc_complete_indir(0, return_buffer, 32, aeq_base);

    u64 return_shadow_buf_addr = cqp_base + 32*cnt;
    __le64 cqe_return_buffer[4];
    // 8 bytes
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);
    temp = 0;
    u8 polarity;
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " create ceq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);

    cnt++;
  } else if (opcode == IRDMA_CQP_OP_GATHER_STATS) {
    u64 return_shadow_buf_addr = cqp_base + 32*cnt;
    __le64 cqe_return_buffer[4];
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);
    temp = 0;
    u8 polarity;
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " create ceq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);

    cnt++;
  } else if (opcode == IRDMA_CQP_OP_CREATE_QP){
    u64 return_shadow_buf_addr = cqp_base + 32*cnt;
    __le64 cqe_return_buffer[4];
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);
    temp = 0;
    u8 polarity;
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    u64 get_polarity;
    get_64bit_val((__le64*)desc, 24, &get_polarity);
    u32 ceq_id = FIELD_GET(IRDMA_CQPSQ_CEQ_CEQID, get_polarity);
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " create ceq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);
    dev.cem.qena_updated(ceq_id);
    cnt++;
  } else if (opcode == IRDMA_CQP_OP_WORK_SCHED_NODE){
    u64 return_shadow_buf_addr = cqp_base + 32*cnt;
    __le64 cqe_return_buffer[4];
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);
    temp = 0;
    u8 polarity;
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    u64 get_polarity;
    get_64bit_val((__le64*)desc, 24, &get_polarity);
    u32 ceq_id = FIELD_GET(IRDMA_CQPSQ_CEQ_CEQID, get_polarity);
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " create ceq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);
    dev.cem.qena_updated(ceq_id);
    cnt++;
  } else if (opcode == IRDMA_CQP_OP_MODIFY_QP){
    u64 return_shadow_buf_addr = cqp_base + 32*cnt;
    __le64 cqe_return_buffer[4];
    set_64bit_val(cqe_return_buffer, 8, dev.cqp.host_cq_pa);
    temp = 0;
    u8 polarity;
    polarity = (u8)1;
    u32 tail = dev.regs.reg_PFPE_CQPTAIL;
    temp = FIELD_PREP(IRDMA_CQ_WQEIDX, tail) | 
            FIELD_PREP(IRDMA_CQ_VALID, polarity);
    
    u64 get_polarity;
    get_64bit_val((__le64*)desc, 24, &get_polarity);
    u32 ceq_id = FIELD_GET(IRDMA_CQPSQ_CEQ_CEQID, get_polarity);
    set_64bit_val(cqe_return_buffer, 24, temp);
    std::cout << " create ceq return buffer addr: "<<return_shadow_buf_addr << logger::endl;
    desc_complete_indir(0, cqe_return_buffer, 32, return_shadow_buf_addr);
    dev.cem.qena_updated(ceq_id);
    cnt++;
  }
  else {
    std::cout << "unhandled opcode is: " << opcode << logger::endl;
  }
  dev.regs.reg_PFPE_CQPTAIL = dev.regs.reg_PFPE_CQPTAIL + 1;
  std::cout << "after process tail is" << dev.regs.reg_PFPE_CQPTAIL << logger::endl;
}

control_queue_pair::dma_data_wb::dma_data_wb(desc_ctx &ctx_, size_t len) : ctx(ctx_){
  data_ = new char[len];
  len_ = len;
}

control_queue_pair::dma_data_wb::~dma_data_wb() {
  delete[]((char *)data_);
}

void control_queue_pair::dma_data_wb::done() {
  delete this;
}

}  // namespace i40e
