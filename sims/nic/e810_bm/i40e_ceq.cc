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

completion_event_manager::completion_event_manager(i40e_bm &dev_, size_t num_qs_)
    : dev(dev_), log("cem", dev_.runner_),num_qs(num_qs_){
  ceqs = new completion_event_queue *[num_qs];
  for (size_t i = 0; i < num_qs; i++) {
    ceqs[i] =
        new completion_event_queue(dev, 0, dev.regs.pf_atqh, dev.regs.pf_atqt);
  }
}

void completion_event_queue::enable(){
  if (!enabled)
    return;
  ctx_fetched();
}

void completion_event_queue::disable() {
  enabled = false;
}

void completion_event_queue::ctx_fetched(){
  initialize();
  enabled = true;
}

void completion_event_queue::initialize(){
  // initialize ceq here
}

void completion_event_manager::qena_updated(uint16_t idx) {
  u32 int_dyn_reg = dev.regs.pfint_dyn_ctln[idx];
  u32 int_ctl_reg = dev.regs.glint_ceqctl[idx];
  u32 msix_idx = FIELD_GET(IRDMA_GLINT_CEQCTL_MSIX_INDX, int_ctl_reg);
  msix_idx = 0x7ff & msix_idx;
	u32 itr_index = FIELD_GET(IRDMA_GLINT_CEQCTL_ITR_INDX, int_ctl_reg);
  itr_index = 0x3 & itr_index;
  bool dyn_ena = 0x1 & int_dyn_reg;
  bool cause_ena = (bool)FIELD_GET(IRDMA_GLINT_CEQCTL_CAUSE_ENA, int_ctl_reg);

  completion_event_queue &ceq = static_cast<completion_event_queue &>(*ceqs[idx]);
  if (!cause_ena || !dyn_ena)
    return;
    // trigger();
  if (!ceq.is_enabled()) {
    // enable and initialize queue here
    ceq.enable();
    ceq.tail_updated(msix_idx, itr_index);
  } else {
    if (msix_idx == 0) {
    dev.regs.pfint_icr0 |=
        1 |
        (1 << (2 + 0));
    }
    ceq.tail_updated(msix_idx, itr_index);
  }
}

void completion_event_manager::reset() {
  for (size_t i = 0; i < num_qs; i++) {
    ceqs[i]->reset();
  }
}

completion_event_queue::completion_event_queue(i40e_bm &dev_, uint64_t ceq_base, uint32_t &reg_head_,
                               uint32_t &reg_tail_)
    : queue_base("ceq", reg_head_, reg_tail_, dev_){
  desc_len = 64;
  ceq_base = ceq_base;
  
}

void completion_event_queue::tail_updated(u32 msix_idx, u32 itr_idx) {
  // trigger();
  dev.SignalInterrupt(msix_idx, itr_idx);
}


void completion_event_queue::trigger() {
  trigger_fetch();
  trigger_process();
  trigger_writeback();
}

void completion_event_queue::trigger_fetch() {
  if (!enabled)
    return;
  
  // prepare data for written back

}

void completion_event_queue::trigger_process() {
  if (!enabled)
    return;
}

void completion_event_queue::trigger_writeback() {
  if (!enabled)
    return;
  
}

completion_event_queue::dma_data_wb::dma_data_wb(completion_event_queue &ceq_): ceq(ceq_) {
  data_ = new char[64];
  len_ = 64;
}

completion_event_queue::dma_data_wb::~dma_data_wb() {
  delete[]((char *)data_);
}

void completion_event_queue::interrupt() {

}

queue_base::desc_ctx &completion_event_queue::desc_ctx_create() {
  
}

void completion_event_queue::writeback_done(uint32_t first_pos, uint32_t cnt) {
  interrupt();
}

void completion_event_queue::dma_data_wb::done() {
  ceq.writeback_done(0, ceq.cnt);
  ceq.trigger();
  delete this;
}

}  // namespace i40e
