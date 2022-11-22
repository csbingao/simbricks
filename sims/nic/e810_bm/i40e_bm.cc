/*
 * Copyright 2021 Max Planck Institute for Software Systems, and
 * National University of Singapore
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "sims/nic/e810_bm/e810_bm.h"

#include <stdlib.h>
#include <string.h>

#include <cassert>
#include <iostream>

#include "lib/simbricks/nicbm/multinic.h"
#include "sims/nic/e810_bm/e810_base_wrapper.h"

namespace i40e {

i40e_bm::i40e_bm()
    : log("i40e", runner_),
      pf_atq(*this, regs.pf_atqba, regs.pf_atqlen, regs.pf_atqh, regs.pf_atqt),
      pf_mbx_atq(*this, regs.pf_mbx_atqba, regs.pf_mbx_atqlen, regs.pf_mbx_atqh, regs.pf_mbx_atqt),
      hmc(*this),
      cqp(*this, regs.reg_PFPE_CCQPHIGH, regs.reg_PFPE_CCQPLOW, regs.pf_atqh, regs.pf_atqt),
      shram(*this),
      lanmgr(*this, NUM_QUEUES),
      cem(*this, NUM_QUEUES) {
  reset(false);
}

i40e_bm::~i40e_bm() {
}

void i40e_bm::SetupIntro(struct SimbricksProtoPcieDevIntro &di) {
  di.bars[BAR_REGS].len = 64 * 1024 * 1024;
  di.bars[BAR_REGS].flags = SIMBRICKS_PROTO_PCIE_BAR_64;
  di.bars[BAR_IO].len = 32;
  di.bars[BAR_IO].flags = SIMBRICKS_PROTO_PCIE_BAR_IO;
  di.bars[BAR_MSIX].len = 64 * 1024;
  di.bars[BAR_MSIX].flags =
      SIMBRICKS_PROTO_PCIE_BAR_64 | SIMBRICKS_PROTO_PCIE_BAR_DUMMY;

  di.pci_vendor_id = E810_INTEL_VENDOR_ID;
  di.pci_device_id = ICE_DEV_ID_E810C_QSFP;
  di.pci_class = 0x02;
  di.pci_subclass = 0x00;
  di.pci_revision = 0x01;
  di.pci_msi_nvecs = 32;

  di.pci_msix_nvecs = 0x80;
  di.pci_msix_table_bar = BAR_MSIX;
  di.pci_msix_pba_bar = BAR_MSIX;
  di.pci_msix_table_offset = 0x0;
  di.pci_msix_pba_offset = 0x8000;
  di.psi_msix_cap_offset = 0x70;
}

void i40e_bm::DmaComplete(nicbm::DMAOp &op) {
  dma_base &dma = dynamic_cast<dma_base &>(op);
#ifdef DEBUG_DEV
  std::cout << "dma_complete(" << &op << ")" << logger::endl;
#endif
  dma.done();
}

void i40e_bm::EthRx(uint8_t port, const void *data, size_t len) {
#ifdef DEBUG_DEV
  std::cout << "i40e: received packet len=" << len << logger::endl;
#endif
  lanmgr.packet_received(data, len);
}

void i40e_bm::RegRead(uint8_t bar, uint64_t addr, void *dest, size_t len) {
  uint32_t *dest_p = reinterpret_cast<uint32_t *>(dest);

  if (len == 4) {
    dest_p[0] = RegRead32(bar, addr);
  } else if (len == 8) {
    dest_p[0] = RegRead32(bar, addr);
    dest_p[1] = RegRead32(bar, addr + 4);
  } else {
    std::cout << "currently we only support 4/8B reads (got " << len << ")"
        << logger::endl;
    abort();
  }
}

uint32_t i40e_bm::RegRead32(uint8_t bar, uint64_t addr) {
  if (bar == BAR_REGS) {
    return reg_mem_read32(addr);
  } else if (bar == BAR_IO) {
    return reg_io_read(addr);
  } else {
    std::cout << "invalid BAR " << (int)bar << logger::endl;
    abort();
  }
}

void i40e_bm::RegWrite(uint8_t bar, uint64_t addr, const void *src,
                       size_t len) {
  const uint32_t *src_p = reinterpret_cast<const uint32_t *>(src);

  if (len == 4) {
    RegWrite32(bar, addr, src_p[0]);
  } else if (len == 8) {
    RegWrite32(bar, addr, src_p[0]);
    RegWrite32(bar, addr + 4, src_p[1]);
  } else {
    std::cout << "currently we only support 4/8B writes (got " << len << ")"
        << logger::endl;
    abort();
  }
}

void i40e_bm::RegWrite32(uint8_t bar, uint64_t addr, uint32_t val) {
  if (bar == BAR_REGS) {
    reg_mem_write32(addr, val);
  } else if (bar == BAR_IO) {
    reg_io_write(addr, val);
  } else {
    std::cout << "invalid BAR " << (int)bar << logger::endl;
    abort();
  }
}

uint32_t i40e_bm::reg_io_read(uint64_t addr) {
  std::cout << "unhandled io read addr=" << addr << logger::endl;
  return 0;
}

void i40e_bm::reg_io_write(uint64_t addr, uint32_t val) {
  std::cout << "unhandled io write addr=" << addr << " val=" << val << logger::endl;
}


uint32_t i40e_bm::reg_mem_read32(uint64_t addr) {
  uint32_t val = 0;

  if (addr >= GLINT_DYN_CTL(0) &&
      addr < GLINT_DYN_CTL(NUM_PFINTS - 1)) {
    val = regs.pfint_dyn_ctln[(addr - GLINT_DYN_CTL(0)) / 4];
  }else if (addr >= QTX_COMM_HEAD(0) &&
             addr <= QTX_COMM_HEAD(16383)) {
    val = regs.qtx_comm_head[(addr - QTX_COMM_HEAD(0)) / 4];
  } else if (addr >= PF0INT_ITR_0(0) &&
             addr <= PF0INT_ITR_0(2047)) {
    val = regs.pfint_itrn[0][(addr - PF0INT_ITR_0(0)) / 4096];
  } else if (addr >= PF0INT_ITR_1(0) &&
             addr <= PF0INT_ITR_1(2047)) {
    val = regs.pfint_itrn[1][(addr - PF0INT_ITR_1(0)) / 4096];
  }else if (addr >= PF0INT_ITR_2(0) &&
             addr <= PF0INT_ITR_2(2047)) {
    val = regs.pfint_itrn[2][(addr - PF0INT_ITR_2(0)) / 4096];
  }else if (addr >= QINT_TQCTL(0) &&
             addr <= QINT_TQCTL(2048 - 1)) {
    val = regs.qint_tqctl[(addr - QINT_TQCTL(0)) / 4];
  } else if (addr >= QINT_RQCTL(0) &&
             addr <= QINT_RQCTL(2047)) {
    val = regs.qint_rqctl[(addr - QINT_RQCTL(0)) / 4];
  } else if (addr >= GLINT_CEQCTL(0) && addr <= GLINT_CEQCTL(2048)) {
    val = regs.glint_ceqctl[(addr - GLINT_CEQCTL(0))];
  } else if (addr >= QRX_CTRL(0) && addr <= QRX_CTRL(2047)) {
    val = regs.QRX_CTRL[(addr - QRX_CTRL(0)) / 4];
  } else if (addr >= QRX_TAIL(0) &&
             addr <= QRX_TAIL(2048 - 1)) {
    val = regs.qrx_tail[(addr - QRX_TAIL(0)) / 4];
  } else if (addr >= GLINT_ITR(0, 0) && addr <= GLINT_ITR(0, 2047)) {
    val = regs.GLINT_ITR0[(addr - GLINT_ITR(0,0))];
  } else if (addr >= GLINT_ITR(1, 0) && addr <= GLINT_ITR(1, 2047)) {
    val = regs.GLINT_ITR1[(addr - GLINT_ITR(1,0))];
  } else if (addr >= GLINT_ITR(2, 0) && addr <= GLINT_ITR(2, 2047)) {
    val = regs.GLINT_ITR2[(addr - GLINT_ITR(2,0))];
  } else if (addr >= QRX_CONTEXT(0, 0) && addr <= QRX_CONTEXT(0, 0)) {
    std::cout << "reading qrx context..." << logger::endl;
    std::cout<< "reading QRX_CONTEXT(0,0)" << logger::endl;
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(0,0))/8192];
  } else if (addr >= QRX_CONTEXT(1, 0) && addr <= QRX_CONTEXT(1, 0)) {
    std::cout<< "reading QRX_CONTEXT(1,0)" << logger::endl;
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(1,0))/8192];
    } else if (addr >= QRX_CONTEXT(2, 0) && addr <= QRX_CONTEXT(2, 0)) {
    std::cout<< "reading QRX_CONTEXT(2,0)" << logger::endl;
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(2,0))/8192];
    } else if (addr >= QRX_CONTEXT(3, 0) && addr <= QRX_CONTEXT(3, 0)) {
    std::cout<< "reading QRX_CONTEXT(3,0)" << logger::endl;
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(3,0))/8192];
    } else if (addr >= QRX_CONTEXT(4, 0) && addr <= QRX_CONTEXT(4, 0)) {
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(4,0))/8192];
    } else if (addr >= QRX_CONTEXT(5, 0) && addr <= QRX_CONTEXT(5, 0)) {
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(5,0))/8192];
    } else if (addr >= QRX_CONTEXT(6, 0) && addr <= QRX_CONTEXT(6, 0)) {
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(6,0))/8192];
    } else if (addr >= QRX_CONTEXT(7, 0) && addr <= QRX_CONTEXT(7, 0)) {
    // QRX_CONTEXT[(addr - QRX_CONTEXT(7,0))/8192] = val;
    val = regs.QRX_CONTEXT[(addr - QRX_CONTEXT(7,0))/8192];
  } else if (addr >= QRXFLXP_CNTXT(0) && addr <= QRXFLXP_CNTXT(2047)) {
    val = regs.QRXFLXP_CNTXT[(addr - QRXFLXP_CNTXT(0)) / 4];
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_0(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_0(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_0(0)) / 4;
    regs.flex_rxdid_0[idx] = val;
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_1(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_1(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_1(0)) / 4;
    val = regs.flex_rxdid_1[idx];
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_2(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_2(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_2(0)) / 4;
    val = regs.flex_rxdid_2[idx];
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_3(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_3(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_3(0)) / 4;
    val = regs.flex_rxdid_3[idx];
  } else if (addr >= GLPRT_BPRCL(0) && addr <= GLPRT_BPRCL(7)){
    size_t idx = (addr - GLPRT_BPRCL(0))/8;
    val = regs.GLPRT_BPRCL[idx];
  } else if (addr >= GLPRT_BPTCL(0) && addr <= GLPRT_BPTCL(7)){
    size_t idx = (addr - GLPRT_BPTCL(0))/8;
    val = regs.GLPRT_BPTCL[idx];
    } else if (addr >= GLPRT_CRCERRS(0) && addr <= GLPRT_CRCERRS(7)){
    size_t idx = (addr - GLPRT_CRCERRS(0))/8;
    val = regs.GLPRT_CRCERRS[idx];
    } else if (addr >= GLPRT_GORCL(0) && addr <= GLPRT_GORCL(7)){
    size_t idx = (addr - GLPRT_GORCL(0))/8;
    val = regs.GLPRT_GORCL[idx];
    } else if (addr >= GLPRT_GOTCL(0) && addr <= GLPRT_GOTCL(7)){
    size_t idx = (addr - GLPRT_GOTCL(0))/8;
    val = regs.GLPRT_GOTCL[idx];
    } else if (addr >= GLPRT_ILLERRC(0) && addr <= GLPRT_ILLERRC(7)){
    size_t idx = (addr - GLPRT_ILLERRC(0))/8;
    val = regs.GLPRT_ILLERRC[idx];
    } else if (addr >= GLPRT_LXOFFRXC(0) && addr <= GLPRT_LXOFFRXC(7)){
    size_t idx = (addr - GLPRT_LXOFFRXC(0))/8;
    val = regs.GLPRT_LXOFFRXC[idx];
    } else if (addr >= GLPRT_LXOFFTXC(0) && addr <= GLPRT_LXOFFTXC(7)){
    size_t idx = (addr - GLPRT_LXOFFTXC(0))/8;
    val = regs.GLPRT_LXOFFTXC[idx];
    } else if (addr >= GLPRT_LXONRXC(0) && addr <= GLPRT_LXONRXC(7)){
    size_t idx = (addr - GLPRT_LXONRXC(0))/8;
    val = regs.GLPRT_LXONRXC[idx];
    } else if (addr >= GLPRT_LXONTXC(0) && addr <= GLPRT_LXONTXC(7)){
    size_t idx = (addr - GLPRT_LXONTXC(0))/8;
    val = regs.GLPRT_LXONTXC[idx];
    } else if (addr >= GLPRT_MLFC(0) && addr <= GLPRT_MLFC(7)){
    size_t idx = (addr - GLPRT_MLFC(0))/8;
    val = regs.GLPRT_MLFC[idx];
    } else if (addr >= GLPRT_MPRCL(0) && addr <= GLPRT_MPRCL(7)){
    size_t idx = (addr - GLPRT_MPRCL(0))/8;
    val = regs.GLPRT_MPRCL[idx];
    } else if (addr >= GLPRT_MPTCL(0) && addr <= GLPRT_MPTCL(7)){
    size_t idx = (addr - GLPRT_MPTCL(0))/8;
    val = regs.GLPRT_MPTCL[idx];
    } else if (addr >= GLPRT_MRFC(0) && addr <= GLPRT_MRFC(7)){
    size_t idx = (addr - GLPRT_MRFC(0))/8;
    val = regs.GLPRT_MRFC[idx];
    } else if (addr >= GLPRT_PRC1023L(0) && addr <= GLPRT_PRC1023L(7)){
    size_t idx = (addr - GLPRT_PRC1023L(0))/8;
    val = regs.GLPRT_PRC1023L[idx];
    } else if (addr >= GLPRT_PRC127L(0) && addr <= GLPRT_PRC127L(7)){
    size_t idx = (addr - GLPRT_PRC127L(0))/8;
    val = regs.GLPRT_PRC127L[idx];
    } else if (addr >= GLPRT_PRC1522L(0) && addr <= GLPRT_PRC1522L(7)){
    size_t idx = (addr - GLPRT_PRC1522L(0))/8;
    val = regs.GLPRT_PRC1522L[idx];
    } else if (addr >= GLPRT_PRC255L(0) && addr <= GLPRT_PRC255L(7)){
    size_t idx = (addr - GLPRT_PRC255L(0))/8;
    val = regs.GLPRT_PRC255L[idx];
    } else if (addr >= GLPRT_PRC511L(0) && addr <= GLPRT_PRC511L(7)){
    size_t idx = (addr - GLPRT_PRC511L(0))/8;
    val = regs.GLPRT_PRC511L[idx];
    } else if (addr >= GLPRT_PRC64L(0) && addr <= GLPRT_PRC64L(7)){
    size_t idx = (addr - GLPRT_PRC64L(0))/8;
    val = regs.GLPRT_PRC64L[idx];
    } else if (addr >= GLPRT_PRC9522L(0) && addr <= GLPRT_PRC9522L(7)){
    size_t idx = (addr - GLPRT_PRC9522L(0))/8;
    val = regs.GLPRT_PRC9522L[idx];
    } else if (addr >= GLPRT_PTC1023L(0) && addr <= GLPRT_PTC1023L(7)){
    size_t idx = (addr - GLPRT_PTC1023L(0))/8;
    val = regs.GLPRT_PTC1023L[idx];
    } else if (addr >= GLPRT_PTC127L(0) && addr <= GLPRT_PTC127L(7)){
    size_t idx = (addr - GLPRT_PTC127L(0))/8;
    val = regs.GLPRT_PTC127L[idx];
    } else if (addr >= GLPRT_PTC1522L(0) && addr <= GLPRT_PTC1522L(7)){
    size_t idx = (addr - GLPRT_PTC1522L(0))/8;
    val = regs.GLPRT_PTC1522L[idx];
    } else if (addr >= GLPRT_PTC255L(0) && addr <= GLPRT_PTC255L(7)){
    size_t idx = (addr - GLPRT_PTC255L(0))/8;
    val = regs.GLPRT_PTC255L[idx];
    } else if (addr >= GLPRT_PTC511L(0) && addr <= GLPRT_PTC511L(7)){
    size_t idx = (addr - GLPRT_PTC511L(0))/8;
    val = regs.GLPRT_PTC511L[idx];
    } else if (addr >= GLPRT_PTC64L(0) && addr <= GLPRT_PTC64L(7)){
    size_t idx = (addr - GLPRT_PTC64L(0))/8;
    val = regs.GLPRT_PTC64L[idx];
    } else if (addr >= GLPRT_PTC9522L(0) && addr <= GLPRT_PTC9522L(7)){
    size_t idx = (addr - GLPRT_PTC9522L(0))/8;
    val = regs.GLPRT_PTC9522L[idx];
    } else if (addr >= GLPRT_RFC(0) && addr <= GLPRT_RFC(7)){
    size_t idx = (addr - GLPRT_RFC(0))/8;
    val = regs.GLPRT_RFC[idx];
    } else if (addr >= GLPRT_RJC(0) && addr <= GLPRT_RJC(7)){
    size_t idx = (addr - GLPRT_RJC(0))/8;
    val = regs.GLPRT_RJC[idx];
    } else if (addr >= GLPRT_RLEC(0) && addr <= GLPRT_RLEC(7)){
    size_t idx = (addr - GLPRT_RLEC(0))/8;
    val = regs.GLPRT_RLEC[idx];
    } else if (addr >= GLPRT_ROC(0) && addr <= GLPRT_ROC(7)){
    size_t idx = (addr - GLPRT_ROC(0))/8;
    val = regs.GLPRT_ROC[idx];
    } else if (addr >= GLPRT_RUC(0) && addr <= GLPRT_RUC(7)){
    size_t idx = (addr - GLPRT_RUC(0))/8;
    val = regs.GLPRT_RUC[idx];
    } else if (addr >= GLPRT_TDOLD(0) && addr <= GLPRT_TDOLD(7)){
    size_t idx = (addr - GLPRT_TDOLD(0))/8;
    val = regs.GLPRT_TDOLD[idx];
    } else if (addr >= GLPRT_UPRCL(0) && addr <= GLPRT_UPRCL(7)){
    size_t idx = (addr - GLPRT_UPRCL(0))/8;
    val = regs.GLPRT_UPRCL[idx];
    } else if (addr >= GLPRT_UPRCL(0) && addr <= GLPRT_UPRCL(7)){
    size_t idx = (addr - GLPRT_UPRCL(0))/8;
    val = regs.GLPRT_UPRCL[idx];
    } else if (addr >= GLV_BPRCL(0) && addr <= GLV_BPRCL(768)){
    size_t idx = (addr - GLV_BPRCL(0))/8;
    val = regs.GLV_BPRCL[idx];
    } else if (addr >= GLV_BPTCL(0) && addr <= GLV_BPTCL(768)){
    size_t idx = (addr - GLV_BPTCL(0))/8;
    val = regs.GLV_BPTCL[idx];
    } else if (addr >= GLV_GORCL(0) && addr <= GLV_GORCL(768)){
    size_t idx = (addr - GLV_GORCL(0))/8;
    val = regs.GLV_GORCL[idx];
    } else if (addr >= GLV_GOTCL(0) && addr <= GLV_GOTCL(768)){
    size_t idx = (addr - GLV_GOTCL(0))/8;
    val = regs.GLV_GOTCL[idx];
    } else if (addr >= GLV_MPRCL(0) && addr <= GLV_MPRCL(768)){
    size_t idx = (addr - GLV_MPRCL(0))/8;
    val = regs.GLV_MPRCL[idx];
    } else if (addr >= GLV_MPTCL(0) && addr <= GLV_MPTCL(768)){
    size_t idx = (addr - GLV_MPTCL(0))/8;
    val = regs.GLV_MPTCL[idx];
    } else if (addr >= GLV_RDPC(0) && addr <= GLV_RDPC(768)){
    size_t idx = (addr - GLV_RDPC(0))/4;
    val = regs.GLV_RDPC[idx];
    } else if (addr >= GLV_TEPC(0) && addr <= GLV_TEPC(768)){
    size_t idx = (addr - GLV_TEPC(0))/4;
    val = regs.GLV_TEPC[idx];
    } else if (addr >= GLV_UPRCL(0) && addr <= GLV_UPRCL(768)){
    size_t idx = (addr - GLV_UPRCL(0))/8;
    val = regs.GLV_UPRCL[idx];
    } else if (addr >= GLV_UPTCL(0) && addr <= GLV_UPTCL(768)){
    size_t idx = (addr - GLV_UPTCL(0))/8;
    val = regs.GLV_UPTCL[idx];
  }
  else {
    std::cout << "read others" << addr << logger::endl;
    switch (addr) {
      case GLINT_CTL:
        val = ((ICE_ITR_GRAN_US << GLINT_CTL_ITR_GRAN_200_S) &
		  GLINT_CTL_ITR_GRAN_200_M) |
		 ((ICE_ITR_GRAN_US << GLINT_CTL_ITR_GRAN_100_S) &
		  GLINT_CTL_ITR_GRAN_100_M) |
		 ((ICE_ITR_GRAN_US << GLINT_CTL_ITR_GRAN_50_S) &
		  GLINT_CTL_ITR_GRAN_50_M) |
		 ((ICE_ITR_GRAN_US << GLINT_CTL_ITR_GRAN_25_S) &
		  GLINT_CTL_ITR_GRAN_25_M);;
        break;
      case PFGEN_CTRL:
        val = 0; /* we always simulate immediate reset */
        break;

      // case I40E_GL_FWSTS:
      //   val = 0;
      //   break;

      // case I40E_GLPCI_CAPSUP:
      //   val = 0;
      //   break;
      case PFPE_CQPDB:
        val = regs.reg_PFPE_CQPDB;
        break;

      case GLPE_CPUSTATUS0:
        val = 0x80;
        break;
      
      case GLPE_CPUSTATUS1:
        val = 0x80;
        break;
      
      case GLPE_CPUSTATUS2:
        val = 0x80;
        break;

      case GLPCI_LBARCTRL:
        val = IRDMA_PE_DB_SIZE_4M << 4;
        break;

      case PFPE_CCQPHIGH:
        val = regs.reg_PFPE_CCQPHIGH;
        break;

      case PFPE_CCQPSTATUS:
        val = regs.reg_PFPE_CCQPSTATUS;
        break;

      case PFPE_CCQPLOW:
        val = regs.reg_PFPE_CCQPLOW;
        break;

      case PFPE_CQPTAIL:
        val = regs.reg_PFPE_CQPTAIL;
        break;

      case GLNVM_ULD:
        val = 0xffffffff;
        break;

      case GLNVM_GENS:
        val = (6 << GLNVM_GENS_SR_SIZE_S);  // shadow ram 64kb
        break;

      case GLNVM_FLA:
        val = GLNVM_GENS_SR_SIZE_M;  // normal flash programming mode
        break;

      case GLGEN_RSTCTL:
        val = regs.glgen_rstctl;
        break;
        
      case GLGEN_STAT:
        val = regs.glgen_stat;
        break;

      // case I40E_GLVFGEN_TIMER:
      //   val = runner_->TimePs() / 1000000;
      //   break;

      // case I40E_PFINT_LNKLST0:
      //   val = regs.pfint_lnklst0;
      //   break;

      case PFINT_OICR_ENA:
        val = regs.pfint_icr0_ena;
        break;

      case PFINT_OICR:
        val = regs.pfint_icr0;
        // read clears
        regs.pfint_icr0 = 0;
        break;

      case PF_VT_PFALLOC_HIF:
        val = 0;  // we don't currently support VFs
        break;

      case GLLAN_RCTL_0:
        val = regs.gllan_rctl_0;
        break;
      case PFHMC_ERRORINFO:
        val = regs.pfhmc_errorinfo;
        break;
      case PFHMC_ERRORDATA:
        val = regs.pfhmc_errordata;
        break;

      // e810 pf
      case PF_FW_ATQBAL:
        val = regs.pf_atqba;
        break;
      case PF_FW_ATQBAH:
        val = regs.pf_atqba >> 32;
        break;
      case PF_FW_ATQLEN:
        val = regs.pf_atqlen;
        break;
      case PF_FW_ATQH:
        val = regs.pf_atqh;
        break;
      case PF_FW_ATQT:
        val = regs.pf_atqt;
        break;

      case PF_FW_ARQBAL:
        val = regs.pf_arqba;
        break;
      case PF_FW_ARQBAH:
        val = regs.pf_arqba >> 32;
        break;
      case PF_FW_ARQLEN:
        val = regs.pf_arqlen;
        break;
      case PF_FW_ARQH:
        val = regs.pf_arqh;
        break;
      case PF_FW_ARQT:
        val = regs.pf_arqt;
        break;
      // e810

      // e810 mailbox
      case PF_MBX_ATQBAL:
        val = regs.pf_mbx_atqba;
        break;
      case PF_MBX_ATQBAH:
        val = regs.pf_mbx_atqba >> 32;
        break;
      case PF_MBX_ATQLEN:
        val = regs.pf_mbx_atqlen;
        break;
      case PF_MBX_ATQH:
        val = regs.pf_mbx_atqh;
        break;
      case PF_MBX_ATQT:
        val = regs.pf_mbx_atqt;
        break;

      case PF_MBX_ARQBAL:
        val = regs.pf_mbx_arqba;
        break;
      case PF_MBX_ARQBAH:
        val = regs.pf_mbx_arqba >> 32;
        break;
      case PF_MBX_ARQLEN:
        val = regs.pf_mbx_arqlen;
        break;
      case PF_MBX_ARQH:
        val = regs.pf_mbx_arqh;
        break;
      case PF_MBX_ARQT:
        val = regs.pf_mbx_arqt;
        break;
      
      case PF_FUNC_RID:
        val = 0;
        break;

      case GLGEN_RSTAT:
        val = 0;
        break;
      // e810

      default:
#ifdef DEBUG_DEV
        std::cout << "unhandled mem read addr=" << addr << logger::endl;
#endif
        break;
    }
  }
  return val;
}

void i40e_bm::reg_mem_write32(uint64_t addr, uint32_t val) {
  if (addr >= GLINT_DYN_CTL(0) &&
      addr <= GLINT_DYN_CTL(NUM_PFINTS - 1)) {
    regs.pfint_dyn_ctln[(addr - GLINT_DYN_CTL(0)) / 4] = val;
  } else if (addr >= QTX_COMM_DBELL(0) && addr <= QTX_COMM_DBELL(2047)){
      size_t idx = (addr - QTX_COMM_DBELL(0))/4;
      regs.QTX_COMM_DBELL[idx] = val;
      regs.qtx_tail[idx] = val;
      lanmgr.tail_updated(idx, false);
  } else if (addr >= QRX_TAIL(0) &&
             addr <= QRX_TAIL(256 - 1)) {
      size_t idx = (addr - QRX_TAIL(0)) / 4;
      regs.qrx_tail[idx] = val & QRX_TAIL_TAIL_M;
      lanmgr.tail_updated(idx, true);
  } else if (addr >= QRX_CTRL(0) && addr <= QRX_CTRL(2047)){
    size_t idx = (addr - QRX_CTRL(0)) / 4;
    regs.QRX_CTRL[idx] = val+4;
    regs.qrx_ena[idx] = val;
    lanmgr.qena_updated(idx, true);
  } else if (addr >= PF0INT_ITR_0(0) &&
             addr <= PF0INT_ITR_0(2047)) {
    regs.pfint_itrn[0][(addr - PF0INT_ITR_0(0)) / 4096] = val;
  } else if (addr >= PF0INT_ITR_1(0) &&
             addr <= PF0INT_ITR_1(2047)) {
    regs.pfint_itrn[1][(addr - PF0INT_ITR_1(0)) / 4096] = val;
  }else if (addr >= PF0INT_ITR_2(0) &&
             addr <= PF0INT_ITR_2(2047)) {
    regs.pfint_itrn[2][(addr - PF0INT_ITR_2(0)) / 4096] = val;
  }else if (addr >= QINT_TQCTL(0) &&
             addr <= QINT_TQCTL(16383)) {
    size_t idx = (addr - QINT_TQCTL(0)) / 4;
    regs.qint_tqctl[idx] = val;
    regs.qtx_ena[idx] = val;
    lanmgr.qena_updated(idx, false);
  } else if (addr >= QINT_RQCTL(0) &&
             addr <= QINT_RQCTL(2048 - 1)) {
    size_t idx = (addr - QINT_RQCTL(0)) / 4;          
    regs.qint_rqctl[idx] = val;
  } else if (addr >= GLINT_CEQCTL(0) && addr <= GLINT_CEQCTL(2018-1)) {
    uint16_t idx = (addr - GLINT_CEQCTL(0)) / 4;
    regs.glint_ceqctl[idx] = val;
    cem.qena_updated(idx);
  } else if (addr >= GLINT_ITR(0, 0) && addr <= GLINT_ITR(0, 2047)) {
    regs.GLINT_ITR0[(addr - GLINT_ITR(0,0))] = val;
  } else if (addr >= GLINT_ITR(1, 0) && addr <= GLINT_ITR(1, 2047)) {
    regs.GLINT_ITR1[(addr - GLINT_ITR(1,0))] = val;
  } else if (addr >= GLINT_ITR(2, 0) && addr <= GLINT_ITR(2, 2047)) {
    regs.GLINT_ITR2[(addr - GLINT_ITR(2,0))] = val;
  } else if (addr >= QRX_CONTEXT(0, 0) && addr <= QRX_CONTEXT(7, 2047)) {
    regs.QRX_CONTEXT[(addr - QRX_CONTEXT(0,0))] = val;
  } else if (addr >= QRXFLXP_CNTXT(0) && addr <= QRXFLXP_CNTXT(2047)) {
    regs.QRXFLXP_CNTXT[(addr - QRXFLXP_CNTXT(0)) / 4] = val;
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_0(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_0(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_0(0)) / 4;
    regs.flex_rxdid_0[idx] = val;
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_1(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_1(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_1(0)) / 4;
    regs.flex_rxdid_1[idx] = val;
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_2(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_2(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_2(0)) / 4;
    regs.flex_rxdid_2[idx] = val;
  } else if (addr >= GLFLXP_RXDID_FLX_WRD_3(0) &&
             addr <= GLFLXP_RXDID_FLX_WRD_3(63)){
    size_t idx = (addr - GLFLXP_RXDID_FLX_WRD_3(0)) / 4;
    regs.flex_rxdid_3[idx] = val;

  } else if (addr >= GLPRT_BPRCL(0) && addr <= GLPRT_BPRCL(7)){
    size_t idx = (addr - GLPRT_BPRCL(0))/8;
    regs.GLPRT_BPRCL[idx] = val;
  } else if (addr >= GLPRT_BPTCL(0) && addr <= GLPRT_BPTCL(7)){
    size_t idx = (addr - GLPRT_BPTCL(0))/8;
    regs.GLPRT_BPTCL[idx] = val;
    } else if (addr >= GLPRT_CRCERRS(0) && addr <= GLPRT_CRCERRS(7)){
    size_t idx = (addr - GLPRT_CRCERRS(0))/8;
    regs.GLPRT_CRCERRS[idx] = val;
    } else if (addr >= GLPRT_GORCL(0) && addr <= GLPRT_GORCL(7)){
    size_t idx = (addr - GLPRT_GORCL(0))/8;
    regs.GLPRT_GORCL[idx] = val;
    } else if (addr >= GLPRT_GOTCL(0) && addr <= GLPRT_GOTCL(7)){
    size_t idx = (addr - GLPRT_GOTCL(0))/8;
    regs.GLPRT_GOTCL[idx] = val;
    } else if (addr >= GLPRT_ILLERRC(0) && addr <= GLPRT_ILLERRC(7)){
    size_t idx = (addr - GLPRT_ILLERRC(0))/8;
    regs.GLPRT_ILLERRC[idx] = val;
    } else if (addr >= GLPRT_LXOFFRXC(0) && addr <= GLPRT_LXOFFRXC(7)){
    size_t idx = (addr - GLPRT_LXOFFRXC(0))/8;
    regs.GLPRT_LXOFFRXC[idx] = val;
    } else if (addr >= GLPRT_LXOFFTXC(0) && addr <= GLPRT_LXOFFTXC(7)){
    size_t idx = (addr - GLPRT_LXOFFTXC(0))/8;
    regs.GLPRT_LXOFFTXC[idx] = val;
    } else if (addr >= GLPRT_LXONRXC(0) && addr <= GLPRT_LXONRXC(7)){
    size_t idx = (addr - GLPRT_LXONRXC(0))/8;
    regs.GLPRT_LXONRXC[idx] = val;
    } else if (addr >= GLPRT_LXONTXC(0) && addr <= GLPRT_LXONTXC(7)){
    size_t idx = (addr - GLPRT_LXONTXC(0))/8;
    regs.GLPRT_LXONTXC[idx] = val;
    } else if (addr >= GLPRT_MLFC(0) && addr <= GLPRT_MLFC(7)){
    size_t idx = (addr - GLPRT_MLFC(0))/8;
    regs.GLPRT_MLFC[idx] = val;
    } else if (addr >= GLPRT_MPRCL(0) && addr <= GLPRT_MPRCL(7)){
    size_t idx = (addr - GLPRT_MPRCL(0))/8;
    regs.GLPRT_MPRCL[idx] = val;
    } else if (addr >= GLPRT_MPTCL(0) && addr <= GLPRT_MPTCL(7)){
    size_t idx = (addr - GLPRT_MPTCL(0))/8;
    regs.GLPRT_MPTCL[idx] = val;
    } else if (addr >= GLPRT_MRFC(0) && addr <= GLPRT_MRFC(7)){
    size_t idx = (addr - GLPRT_MRFC(0))/8;
    regs.GLPRT_MRFC[idx] = val;
    } else if (addr >= GLPRT_PRC1023L(0) && addr <= GLPRT_PRC1023L(7)){
    size_t idx = (addr - GLPRT_PRC1023L(0))/8;
    regs.GLPRT_PRC1023L[idx] = val;
    } else if (addr >= GLPRT_PRC127L(0) && addr <= GLPRT_PRC127L(7)){
    size_t idx = (addr - GLPRT_PRC127L(0))/8;
    regs.GLPRT_PRC127L[idx] = val;
    } else if (addr >= GLPRT_PRC1522L(0) && addr <= GLPRT_PRC1522L(7)){
    size_t idx = (addr - GLPRT_PRC1522L(0))/8;
    regs.GLPRT_PRC1522L[idx] = val;
    } else if (addr >= GLPRT_PRC255L(0) && addr <= GLPRT_PRC255L(7)){
    size_t idx = (addr - GLPRT_PRC255L(0))/8;
    regs.GLPRT_PRC255L[idx] = val;
    } else if (addr >= GLPRT_PRC511L(0) && addr <= GLPRT_PRC511L(7)){
    size_t idx = (addr - GLPRT_PRC511L(0))/8;
    regs.GLPRT_PRC511L[idx] = val;
    } else if (addr >= GLPRT_PRC64L(0) && addr <= GLPRT_PRC64L(7)){
    size_t idx = (addr - GLPRT_PRC64L(0))/8;
    regs.GLPRT_PRC64L[idx] = val;
    } else if (addr >= GLPRT_PRC9522L(0) && addr <= GLPRT_PRC9522L(7)){
    size_t idx = (addr - GLPRT_PRC9522L(0))/8;
    regs.GLPRT_PRC9522L[idx] = val;
    } else if (addr >= GLPRT_PTC1023L(0) && addr <= GLPRT_PTC1023L(7)){
    size_t idx = (addr - GLPRT_PTC1023L(0))/8;
    regs.GLPRT_PTC1023L[idx] = val;
    } else if (addr >= GLPRT_PTC127L(0) && addr <= GLPRT_PTC127L(7)){
    size_t idx = (addr - GLPRT_PTC127L(0))/8;
    regs.GLPRT_PTC127L[idx] = val;
    } else if (addr >= GLPRT_PTC1522L(0) && addr <= GLPRT_PTC1522L(7)){
    size_t idx = (addr - GLPRT_PTC1522L(0))/8;
    regs.GLPRT_PTC1522L[idx] = val;
    } else if (addr >= GLPRT_PTC255L(0) && addr <= GLPRT_PTC255L(7)){
    size_t idx = (addr - GLPRT_PTC255L(0))/8;
    regs.GLPRT_PTC255L[idx] = val;
    } else if (addr >= GLPRT_PTC511L(0) && addr <= GLPRT_PTC511L(7)){
    size_t idx = (addr - GLPRT_PTC511L(0))/8;
    regs.GLPRT_PTC511L[idx] = val;
    } else if (addr >= GLPRT_PTC64L(0) && addr <= GLPRT_PTC64L(7)){
    size_t idx = (addr - GLPRT_PTC64L(0))/8;
    regs.GLPRT_PTC64L[idx] = val;
    } else if (addr >= GLPRT_PTC9522L(0) && addr <= GLPRT_PTC9522L(7)){
    size_t idx = (addr - GLPRT_PTC9522L(0))/8;
    regs.GLPRT_PTC9522L[idx] = val;
    } else if (addr >= GLPRT_RFC(0) && addr <= GLPRT_RFC(7)){
    size_t idx = (addr - GLPRT_RFC(0))/8;
    regs.GLPRT_RFC[idx] = val;
    } else if (addr >= GLPRT_RJC(0) && addr <= GLPRT_RJC(7)){
    size_t idx = (addr - GLPRT_RJC(0))/8;
    regs.GLPRT_RJC[idx] = val;
    } else if (addr >= GLPRT_RLEC(0) && addr <= GLPRT_RLEC(7)){
    size_t idx = (addr - GLPRT_RLEC(0))/8;
    regs.GLPRT_RLEC[idx] = val;
    } else if (addr >= GLPRT_ROC(0) && addr <= GLPRT_ROC(7)){
    size_t idx = (addr - GLPRT_ROC(0))/8;
    regs.GLPRT_ROC[idx] = val;
    } else if (addr >= GLPRT_RUC(0) && addr <= GLPRT_RUC(7)){
    size_t idx = (addr - GLPRT_RUC(0))/8;
    regs.GLPRT_RUC[idx] = val;
    } else if (addr >= GLPRT_TDOLD(0) && addr <= GLPRT_TDOLD(7)){
    size_t idx = (addr - GLPRT_TDOLD(0))/8;
    regs.GLPRT_TDOLD[idx] = val;
    } else if (addr >= GLPRT_UPRCL(0) && addr <= GLPRT_UPRCL(7)){
    size_t idx = (addr - GLPRT_UPRCL(0))/8;
    regs.GLPRT_UPRCL[idx] = val;
    } else if (addr >= GLPRT_UPRCL(0) && addr <= GLPRT_UPRCL(7)){
    size_t idx = (addr - GLPRT_UPRCL(0))/8;
    regs.GLPRT_UPRCL[idx] = val;
    } else if (addr >= GLV_BPRCL(0) && addr <= GLV_BPRCL(7)){
    size_t idx = (addr - GLV_BPRCL(0))/8;
    regs.GLV_BPRCL[idx] = val;
    } else if (addr >= GLV_BPTCL(0) && addr <= GLV_BPTCL(7)){
    size_t idx = (addr - GLV_BPTCL(0))/8;
    regs.GLV_BPTCL[idx] = val;
    } else if (addr >= GLV_GORCL(0) && addr <= GLV_GORCL(7)){
    size_t idx = (addr - GLV_GORCL(0))/8;
    regs.GLV_GORCL[idx] = val;
    } else if (addr >= GLV_GOTCL(0) && addr <= GLV_GOTCL(7)){
    size_t idx = (addr - GLV_GOTCL(0))/8;
    regs.GLV_GOTCL[idx] = val;
    } else if (addr >= GLV_MPRCL(0) && addr <= GLV_MPRCL(7)){
    size_t idx = (addr - GLV_MPRCL(0))/8;
    regs.GLV_MPRCL[idx] = val;
    } else if (addr >= GLV_MPTCL(0) && addr <= GLV_MPTCL(7)){
    size_t idx = (addr - GLV_MPTCL(0))/8;
    regs.GLV_MPTCL[idx] = val;
    } else if (addr >= GLV_RDPC(0) && addr <= GLV_RDPC(7)){
    size_t idx = (addr - GLV_RDPC(0))/4;
    regs.GLV_RDPC[idx] = val;
    } else if (addr >= GLV_TEPC(0) && addr <= GLV_TEPC(7)){
    size_t idx = (addr - GLV_TEPC(0))/4;
    regs.GLV_TEPC[idx] = val;
    } else if (addr >= GLV_UPRCL(0) && addr <= GLV_UPRCL(7)){
    size_t idx = (addr - GLV_UPRCL(0))/8;
    regs.GLV_UPRCL[idx] = val;
    } else if (addr >= GLV_UPTCL(0) && addr <= GLV_UPTCL(7)){
    size_t idx = (addr - GLV_UPTCL(0))/8;
    regs.GLV_UPTCL[idx] = val;
    } else {
      std::cout << "write others " << addr << logger::endl;
      std::cout << "write others value " << val << logger::endl;
    switch (addr) {
      case PFPE_CCQPSTATUS:
        regs.reg_PFPE_CCQPSTATUS = val;
        break;

      case PFPE_CQPDB:
        regs.reg_PFPE_CQPDB = val;
        if (val == 0){
          break;
        }
        if ((0x3ff & val) > regs.reg_PFPE_CQPTAIL){
          cqp.reg_updated();
        }
        
        break;

      case PFPE_CCQPHIGH:
        regs.reg_PFPE_CCQPHIGH = val;
        break;
      
      case PFPE_CCQPLOW:
        regs.reg_PFPE_CCQPLOW = val;
        
        cqp.create_cqp();
        break;

      case PFPE_CQPTAIL:
        regs.reg_PFPE_CQPTAIL = val;
        break;

      case PFGEN_CTRL:
        if ((val & PFGEN_CTRL_PFSWR_M) == PFGEN_CTRL_PFSWR_M)
          reset(true);
        break;

      case GLGEN_STAT:
        regs.glgen_stat = val;
        break;

      case GLINT_CTL:
        regs.glint_ctl = val;
        break;

      case GLGEN_RSTCTL:
        regs.glgen_rstctl = val;
        break;

      case GLNVM_GENS:
        regs.glnvm_srctl = val;
        shram.reg_updated();
        break;

      case PFINT_OICR_ENA:
        regs.pfint_icr0_ena = val;
        break;
      case PFINT_OICR:
        regs.pfint_icr0 = val;
        break;

      case PF_FW_ATQBAL:
        regs.pf_atqba = val | (regs.pf_atqba & 0xffffffff00000000ULL);
        pf_atq.reg_updated();
        break;
      case PF_FW_ATQBAH:
        regs.pf_atqba = ((uint64_t)val << 32) | (regs.pf_atqba & 0xffffffffULL);
        pf_atq.reg_updated();
        break;
      case PF_FW_ATQLEN:
        regs.pf_atqlen = val;
        pf_atq.reg_updated();
        break;
      case PF_FW_ATQH:
        regs.pf_atqh = val;
        pf_atq.reg_updated();
        break;
      case PF_FW_ATQT:
        regs.pf_atqt = val;
        pf_atq.reg_updated();
        break;

      case PF_FW_ARQBAL:
        regs.pf_arqba = val | (regs.pf_atqba & 0xffffffff00000000ULL);
        break;
      case PF_FW_ARQBAH:
        regs.pf_arqba = ((uint64_t)val << 32) | (regs.pf_arqba & 0xffffffffULL);
        break;
      case PF_FW_ARQLEN:
        regs.pf_arqlen = val;
        break;
      case PF_FW_ARQH:
        regs.pf_arqh = val;
        break;
      case PF_FW_ARQT:
        regs.pf_arqt = val;
        break;

      // mailbox
      case PF_MBX_ATQBAL:
        regs.pf_mbx_atqba = val | (regs.pf_mbx_atqba & 0xffffffff00000000ULL);
        pf_atq.reg_updated();
        break;
      case PF_MBX_ATQBAH:
        regs.pf_mbx_atqba = ((uint64_t)val << 32) | (regs.pf_mbx_atqba & 0xffffffffULL);
        pf_atq.reg_updated();
        break;
      case PF_MBX_ATQLEN:
        regs.pf_mbx_atqlen = val;
        pf_atq.reg_updated();
        break;
      case PF_MBX_ATQH:
        regs.pf_mbx_atqh = val;
        pf_atq.reg_updated();
        break;
      case PF_MBX_ATQT:
        regs.pf_mbx_atqt = val;
        pf_atq.reg_updated();
        break;

      case PF_MBX_ARQBAL:
        regs.pf_mbx_arqba = val | (regs.pf_mbx_atqba & 0xffffffff00000000ULL);
        break;
      case PF_MBX_ARQBAH:
        regs.pf_mbx_arqba = ((uint64_t)val << 32) | (regs.pf_mbx_arqba & 0xffffffffULL);
        break;
      case PF_MBX_ARQLEN:
        regs.pf_mbx_arqlen = val;
        break;
      case PF_MBX_ARQH:
        regs.pf_mbx_arqh = val;
        break;
      case PF_MBX_ARQT:
        regs.pf_mbx_arqt = val;
        break;


      default:
#ifdef DEBUG_DEV
        std::cout << "unhandled mem write addr=" << addr << " val=" << val
            << logger::endl;
#endif
        break;
    }
  }
}

void i40e_bm::Timed(nicbm::TimedEvent &ev) {
  int_ev &iev = *((int_ev *)&ev);
#ifdef DEBUG_DEV
  std::cout << "timed_event: triggering interrupt (" << iev.vec << ")"
      << logger::endl;
#endif
  iev.armed = false;

  if (int_msix_en_) {
    runner_->MsiXIssue(iev.vec);
  } else if (iev.vec > 0) {
    std::cout << "timed_event: MSI-X disabled, but vec != 0" << logger::endl;
    abort();
  } else {
    runner_->MsiIssue(0);
  }
}

void i40e_bm::SignalInterrupt(uint16_t vec, uint8_t itr) {
  int_ev &iev = intevs[vec];

  uint64_t mindelay;
  if (itr <= 2) {
    // itr 0-2
    if (vec == 0)
      mindelay = regs.GLINT_ITR0[itr];
    else if (vec == 1)
      mindelay = regs.GLINT_ITR1[itr];
    else
      mindelay = regs.GLINT_ITR2[itr];
    mindelay *= 2000000ULL;
  } else if (itr == 3) {
    // noitr
    mindelay = 0;
  } else {
    std::cout << "signal_interrupt() invalid itr (" << itr << ")" << logger::endl;
    abort();
  }

  uint64_t curtime = runner_->TimePs();
  uint64_t newtime = curtime + mindelay;
  if (iev.armed && iev.time_ <= newtime) {
    // already armed and this is not scheduled sooner
#ifdef DEBUG_DEV
    std::cout << "signal_interrupt: vec " << vec << " already scheduled"
        << logger::endl;
#endif
    return;
  } else if (iev.armed) {
    // need to reschedule
    runner_->EventCancel(iev);
  }

  iev.armed = true;
  iev.time_ = newtime;

#ifdef DEBUG_DEV
  std::cout << "signal_interrupt: scheduled vec " << vec << " for time=" << newtime
      << " (itr " << itr << ")" << logger::endl;
#endif

  runner_->EventSchedule(iev);
}

void i40e_bm::reset(bool indicate_done) {
#ifdef DEBUG_DEV
  std::cout << "reset triggered" << logger::endl;
#endif

  pf_atq.reset();
  hmc.reset();
  lanmgr.reset();

  memset(&regs, 0, sizeof(regs));
  // if (indicate_done)
  //   regs.glnvm_srctl = I40E_GLNVM_SRCTL_DONE_MASK;

  for (uint16_t i = 0; i < NUM_PFINTS; i++) {
    intevs[i].vec = i;
    if (intevs[i].armed) {
      runner_->EventCancel(intevs[i]);
      intevs[i].armed = false;
    }
    intevs[i].time_ = 0;
  }

  // add default hash key
  regs.pfqf_hkey[0] = 0xda565a6d;
  regs.pfqf_hkey[1] = 0xc20e5b25;
  regs.pfqf_hkey[2] = 0x3d256741;
  regs.pfqf_hkey[3] = 0xb08fa343;
  regs.pfqf_hkey[4] = 0xcb2bcad0;
  regs.pfqf_hkey[5] = 0xb4307bae;
  regs.pfqf_hkey[6] = 0xa32dcb77;
  regs.pfqf_hkey[7] = 0x0cf23080;
  regs.pfqf_hkey[8] = 0x3bb7426a;
  regs.pfqf_hkey[9] = 0xfa01acbe;
  regs.pfqf_hkey[10] = 0x0;
  regs.pfqf_hkey[11] = 0x0;
  regs.pfqf_hkey[12] = 0x0;

  regs.glrpb_ghw = 0xF2000;
  regs.glrpb_phw = 0x1246;
  regs.glrpb_plw = 0x0846;
}

shadow_ram::shadow_ram(i40e_bm &dev_) : dev(dev_), log("sram", dev_.runner_) {
}

void shadow_ram::reg_updated() {
  uint32_t val = dev.regs.glnvm_srctl;
  uint32_t addr;
  bool is_write;
  return;
//   if (!(val & I40E_GLNVM_SRCTL_START_MASK))
//     return;

//   addr = (val & I40E_GLNVM_SRCTL_ADDR_MASK) >> I40E_GLNVM_SRCTL_ADDR_SHIFT;
//   is_write = (val & I40E_GLNVM_SRCTL_WRITE_MASK);

// #ifdef DEBUG_DEV
//   std::cout << "shadow ram op addr=" << addr << " w=" << is_write << logger::endl;
// #endif

//   if (is_write) {
//     write(addr, (dev.regs.glnvm_srdata & I40E_GLNVM_SRDATA_WRDATA_MASK) >>
//                     I40E_GLNVM_SRDATA_WRDATA_SHIFT);
//   } else {
//     dev.regs.glnvm_srdata &= ~I40E_GLNVM_SRDATA_RDDATA_MASK;
//     dev.regs.glnvm_srdata |= ((uint32_t)read(addr))
//                              << I40E_GLNVM_SRDATA_RDDATA_SHIFT;
//   }

//   dev.regs.glnvm_srctl &= ~I40E_GLNVM_SRCTL_START_MASK;
//   dev.regs.glnvm_srctl |= I40E_GLNVM_SRCTL_DONE_MASK;
}

uint16_t shadow_ram::read(uint16_t addr) {
  switch (addr) {
    /* for any of these hopefully return 0 should be fine */
    /* they are read by drivers but not used */
    case I40E_SR_NVM_DEV_STARTER_VERSION:
    case I40E_SR_NVM_EETRACK_LO:
    case I40E_SR_NVM_EETRACK_HI:
    case I40E_SR_BOOT_CONFIG_PTR:
      return 0;

    case I40E_SR_NVM_CONTROL_WORD:
      return (1 << I40E_SR_CONTROL_WORD_1_SHIFT);

    case I40E_SR_SW_CHECKSUM_WORD:
      return 0xbaba;

    default:
#ifdef DEBUG_DEV
      std::cout << "TODO shadow memory read addr=" << addr << logger::endl;
#endif
      break;
  }

  return 0;
}

void shadow_ram::write(uint16_t addr, uint16_t val) {
#ifdef DEBUG_DEV
  std::cout << "TODO shadow memory write addr=" << addr << " val=" << val
      << logger::endl;
#endif
}

int_ev::int_ev() {
  armed = false;
  time_ = 0;
}

}  // namespace i40e

class i40e_factory : public nicbm::MultiNicRunner::DeviceFactory {
  public:
    virtual nicbm::Runner::Device &create() override {
      return *new i40e::i40e_bm;
    }
};

int main(int argc, char *argv[]) {
  i40e_factory fact;
  nicbm::MultiNicRunner mr(fact);
  return mr.RunMain(argc, argv);
}
