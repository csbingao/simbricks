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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <cassert>
#include <iostream>
using namespace std;
#include "sims/nic/e810_bm/i40e_base_wrapper.h"
#include "sims/nic/e810_bm/i40e_bm.h"
// #include "sims/nic/e810_bm/base/ice_adminq_cmd.h"
#include <bits/stdc++.h>
namespace i40e {

queue_admin_tx::queue_admin_tx(i40e_bm &dev_, uint64_t &reg_base_,
                               uint32_t &reg_len_, uint32_t &reg_head_,
                               uint32_t &reg_tail_)
    : queue_base("atx", reg_head_, reg_tail_, dev_),
      reg_base(reg_base_),
      reg_len(reg_len_) {
  desc_len = 32;
  ctxs_init();
}

queue_base::desc_ctx &queue_admin_tx::desc_ctx_create() {
  return *new admin_desc_ctx(*this, dev);
}

void queue_admin_tx::reg_updated() {
  base = reg_base;
  len = (reg_len & I40E_GL_ATQLEN_ATQLEN_MASK) >> I40E_GL_ATQLEN_ATQLEN_SHIFT;

  if (!enabled && (reg_len & I40E_GL_ATQLEN_ATQENABLE_MASK)) {
#ifdef DEBUG_ADMINQ
    cout << " enable base=" << base << " len=" << len << logger::endl;
#endif
    enabled = true;
  } else if (enabled && !(reg_len & I40E_GL_ATQLEN_ATQENABLE_MASK)) {
    cout << " disable" << logger::endl;
#ifdef DEBUG_ADMINQ
    cout << " disable" << logger::endl;
#endif
    enabled = false;
  }

  queue_base::reg_updated();
}

queue_admin_tx::admin_desc_ctx::admin_desc_ctx(queue_admin_tx &queue_,
                                               i40e_bm &dev_)
    : i40e::queue_base::desc_ctx(queue_), aq(queue_), dev(dev_) {
  d = reinterpret_cast<struct i40e_aq_desc *>(desc);
}

void queue_admin_tx::admin_desc_ctx::data_written(uint64_t addr, size_t len) {
  processed();
}

void queue_admin_tx::admin_desc_ctx::desc_compl_prepare(uint16_t retval,
                                                        uint16_t extra_flags) {
  d->flags &= ~0x1ff;
  d->flags |= I40E_AQ_FLAG_DD | I40E_AQ_FLAG_CMP | extra_flags;
  if (retval)
    d->flags |= I40E_AQ_FLAG_ERR;
  d->retval = retval;

#ifdef DEBUG_ADMINQ
  cout <<  " desc_compl_prepare index=" << index << " retval=" << retval
            << logger::endl;
#endif
}

void queue_admin_tx::admin_desc_ctx::desc_complete(uint16_t retval,
                                                   uint16_t extra_flags) {
  desc_compl_prepare(retval, extra_flags);
  processed();
}

void queue_admin_tx::admin_desc_ctx::desc_complete_indir(uint16_t retval,
                                                         const void *data,
                                                         size_t len,
                                                         uint16_t extra_flags,
                                                         bool ignore_datalen) {
  if (!ignore_datalen && len > d->datalen) {
    cout <<  "queue_admin_tx::desc_complete_indir: data too long (" << len
              << ") got buffer for (" << d->datalen << ")" << logger::endl;
    abort();
  }
  d->datalen = len;

  desc_compl_prepare(retval, extra_flags);

  uint64_t addr = d->params.external.addr_low |
                  (((uint64_t)d->params.external.addr_high) << 32);
  data_write(addr, len, data);
}

void queue_admin_tx::admin_desc_ctx::prepare() {
  if ((d->flags & I40E_AQ_FLAG_RD)) {
    uint64_t addr = d->params.external.addr_low |
                    (((uint64_t)d->params.external.addr_high) << 32);
#ifdef DEBUG_ADMINQ
    cout <<  " desc with buffer opc=" << d->opcode << " addr=" << addr
              << logger::endl;
#endif
    data_fetch(addr, d->datalen);
  } else {
    prepared();
  }
}

void queue_admin_tx::admin_desc_ctx::process() {
  cout << " opcode " << d->opcode << " is processing" << logger::endl;
#ifdef DEBUG_ADMINQ
  cout <<  " descriptor " << index << " fetched" << logger::endl;
#endif

  if (d->opcode == ice_aqc_opc_get_ver) {
#ifdef DEBUG_ADMINQ
    cout <<  "  get version" << logger::endl;
#endif
    struct ice_aqc_get_ver *gv =
        reinterpret_cast<struct ice_aqc_get_ver *>(d->params.raw);
    gv->rom_ver = 0;
    gv->fw_build = 0;
    gv->fw_major = 0;
    gv->fw_minor = 0;
    gv->api_major = EXP_FW_API_VER_MAJOR;
    gv->api_minor = EXP_FW_API_VER_MINOR;

    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_req_res) {
#ifdef DEBUG_ADMINQ
    cout <<  "  request resource" << logger::endl;
#endif
    struct ice_aqc_req_res *rr =
        reinterpret_cast<struct ice_aqc_req_res *>(d->params.raw);
    rr->timeout = 180000;
#ifdef DEBUG_ADMINQ
    cout <<  "    res_id=" << rr->res_id << logger::endl;
    cout <<  "    res_nu=" << rr->res_number << logger::endl;
#endif
    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_release_res) {
#ifdef DEBUG_ADMINQ
    cout <<  "  release resource" << logger::endl;
#endif
#ifdef DEBUG_ADMINQ
    struct ice_aqc_req_res *rr =
        reinterpret_cast<struct ice_aqc_req_res *>(d->params.raw);
    cout <<  "    res_id=" << rr->res_id << logger::endl;
    cout <<  "    res_nu=" << rr->res_number << logger::endl;
#endif
    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_clear_pxe_mode) {
#ifdef DEBUG_ADMINQ
    cout <<  "  clear PXE mode" << logger::endl;
#endif
    dev.regs.gllan_rctl_0 &= ~I40E_GLLAN_RCTL_0_PXE_MODE_MASK;
    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_list_dev_caps ||
             d->opcode == ice_aqc_opc_list_func_caps) {
#ifdef DEBUG_ADMINQ
    cout <<  "  get dev/fun caps" << logger::endl;
#endif
    struct ice_aqc_list_caps *lc =
        reinterpret_cast<struct ice_aqc_list_caps *>(d->params.raw);

    struct ice_aqc_list_caps_elem caps[] = {
        {ICE_AQC_CAPS_VALID_FUNCTIONS, 1, 0, dev.NUM_VSIS, 8, 0, {}},
        {ICE_AQC_CAPS_RSS, 1, 0, 2048, 8, 0, {}},
        {ICE_AQC_CAPS_RXQS, 1, 0, dev.NUM_QUEUES, 0, 0, {}},
        {ICE_AQC_CAPS_TXQS, 1, 0, dev.NUM_QUEUES, 0, 0, {}},
        {ICE_AQC_CAPS_MSIX, 1, 0, dev.NUM_PFINTS, 0, 0, {}},
        {ICE_AQC_CAPS_VSI, 1, 0, dev.NUM_VSIS, 0, 0, {}},
        {ICE_AQC_CAPS_DCB, 1, 0, 1, 8, 1, {}},
    };
    size_t num_caps = sizeof(caps) / sizeof(caps[0]);

    if (sizeof(caps) <= d->datalen) {
#ifdef DEBUG_ADMINQ
      cout <<  "    data fits" << logger::endl;
#endif
      // data fits within the buffer
      lc->count = num_caps;
      desc_complete_indir(0, caps, sizeof(caps));
    } else {
#ifdef DEBUG_ADMINQ
      cout <<  "    data doesn't fit" << logger::endl;
#endif
      // data does not fit
      d->datalen = sizeof(caps);
      desc_complete(ICE_AQ_RC_ENOMEM);
    }
  } else if (d->opcode == ice_aqc_opc_lldp_stop) {
#ifdef DEBUG_ADMINQ
    cout <<  "  lldp stop" << logger::endl;
#endif
    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_manage_mac_read) {
#ifdef DEBUG_ADMINQ
    cout <<  "  read mac" << logger::endl;
#endif
    struct ice_aqc_manage_mac_read *ar =
        reinterpret_cast<struct ice_aqc_manage_mac_read *>(d->params.raw);
    ar->num_addr = 1;
    struct ice_aqc_manage_mac_read_resp ard;
    uint64_t mac = dev.runner_->GetMacAddr();
#ifdef DEBUG_ADMINQ
    cout <<  "    mac = " << mac << logger::endl;
#endif
    ard.addr_type = ICE_AQC_MAN_MAC_ADDR_TYPE_LAN;
    memcpy(ard.mac_addr, &mac, 6);

    ar->flags = ICE_AQC_MAN_MAC_LAN_ADDR_VALID | ICE_AQC_MAN_MAC_PORT_ADDR_VALID;
    desc_complete_indir(0, &ard, sizeof(ard));
  } else if (d->opcode == ice_aqc_opc_get_phy_caps) {
#ifdef DEBUG_ADMINQ
    cout <<  "  get phy abilities" << logger::endl;
#endif
    struct ice_aqc_get_phy_caps_data par;
    memset(&par, 0, sizeof(par));

    par.phy_type_low = ICE_PHY_TYPE_LOW_40GBASE_CR4;
    par.phy_type_high = ICE_PHY_TYPE_HIGH_100G_CAUI2_AOC_ACC;
    par.caps = I40E_AQ_PHY_LINK_ENABLED | I40E_AQ_PHY_AN_ENABLED;
    par.eee_cap = ICE_AQC_PHY_EEE_EN_40GBASE_KR4;

    d->params.external.param0 = 0;
    d->params.external.param1 = 0;

    desc_complete_indir(0, &par, sizeof(par));
  } else if (d->opcode == ice_aqc_opc_get_link_status) {
#ifdef DEBUG_ADMINQ
    cout <<  "  link status" << logger::endl;
#endif
    struct ice_aqc_get_link_status *gls =
        reinterpret_cast<struct ice_aqc_get_link_status *>(d->params.raw);

    gls->cmd_flags &= ICE_AQ_LSE_IS_ENABLED;  // should actually return
                                                   // status of link status
                                                   // notification
    struct ice_aqc_get_link_status_data link_status_data;
    // link_status_data.topo_media_conflict = BIT(3);
    link_status_data.phy_type_low = ICE_PHY_TYPE_LOW_40GBASE_CR4;
    // link_status_data.phy_type_high = 
    link_status_data.link_speed = ICE_AQ_LINK_SPEED_40GB;
    link_status_data.link_info = ICE_AQ_LINK_UP | ICE_AQ_LINK_UP_PORT |
                     ICE_AQ_MEDIA_AVAILABLE | ICE_AQ_SIGNAL_DETECT;
    // might need qualified module
    link_status_data.an_info = ICE_AQ_AN_COMPLETED | ICE_AQ_LP_AN_ABILITY;
    link_status_data.ext_info = 0;
    // link_status_data.loopback = I40E_AQ_LINK_POWER_CLASS_4 << I40E_AQ_PWR_CLASS_SHIFT_LB;
    link_status_data.max_frame_size = dev.MAX_MTU;
    // link_status_data.config = I40E_AQ_CONFIG_CRC_ENA;

    desc_complete_indir(0, &link_status_data, sizeof(link_status_data));
  } else if (d->opcode == ice_aqc_opc_get_sw_cfg) {
#ifdef DEBUG_ADMINQ
    cout <<  "  get switch config" << logger::endl;
#endif
    struct ice_aqc_get_sw_cfg *sw =
        reinterpret_cast<struct ice_aqc_get_sw_cfg *>(d->params.raw);
    struct ice_aqc_get_sw_cfg_resp hr;
    /* Not sure why dpdk doesn't like this?
    struct i40e_aqc_switch_config_element_resp els[] = {
        // EMC
        { I40E_AQ_SW_ELEM_TYPE_EMP, I40E_AQ_SW_ELEM_REV_1, 1, 513, 0, {},
            I40E_AQ_CONN_TYPE_REGULAR, 0, 0},
        // MAC
        { I40E_AQ_SW_ELEM_TYPE_MAC, I40E_AQ_SW_ELEM_REV_1, 2, 0, 0, {},
            I40E_AQ_CONN_TYPE_REGULAR, 0, 0},
        // PF
        { I40E_AQ_SW_ELEM_TYPE_PF, I40E_AQ_SW_ELEM_REV_1, 16, 512, 0, {},
            I40E_AQ_CONN_TYPE_REGULAR, 0, 0},
        // VSI PF
        { I40E_AQ_SW_ELEM_TYPE_VSI, I40E_AQ_SW_ELEM_REV_1, 512, 2, 16, {},
            I40E_AQ_CONN_TYPE_REGULAR, 0, 0},
        // VSI PF
        { I40E_AQ_SW_ELEM_TYPE_VSI, I40E_AQ_SW_ELEM_REV_1, 513, 2, 1, {},
            I40E_AQ_CONN_TYPE_REGULAR, 0, 0},
    };*/
    struct ice_aqc_get_sw_cfg_resp els[] = {
        // VSI PF
        {ICE_AQC_GET_SW_CONF_RESP_PHYS_PORT << ICE_AQC_GET_SW_CONF_RESP_TYPE_S || 1,
         0,
         0},
    };

    // find start idx
    size_t cnt = sizeof(els) / sizeof(els[0]);
    size_t first = 0;
    // for (first = 0; first < cnt && els[first].seid < sw->seid; first++) {
    // }

    // figure out how many fit in the buffer
    size_t max = (d->datalen - sizeof(hr)) / sizeof(els[0]);
    size_t report = cnt - first;
    sw->num_elems = 1;

    // prepare header
    // memset(&hr, 0, sizeof(hr));
// #ifdef DEBUG_ADMINQ
//     cout <<  "    report=" << report << " cnt=" << cnt
//               << "  seid=" << sw->seid << logger::endl;
// #endif

    // create temporary contiguous buffer
    size_t buflen = sizeof(hr) + sizeof(els[0]) * report;
    uint8_t buf[buflen];
    memcpy(buf, &hr, sizeof(hr));
    memcpy(buf + sizeof(hr), els + first, sizeof(els[0]) * report);

    desc_complete_indir(0, buf, buflen);
  } else if (d->opcode == ice_aqc_opc_get_pkg_info_list){
    struct ice_aqc_get_pkg_info_resp *v =
        reinterpret_cast<struct ice_aqc_get_pkg_info_resp *>(
                d->params.raw);
    struct ice_aqc_get_pkg_info_resp *get_pkg_info = reinterpret_cast<struct ice_aqc_get_pkg_info_resp*> (data);
    get_pkg_info[0].count = 1;
    get_pkg_info[0].pkg_info[0].is_active = 1;
    get_pkg_info[0].pkg_info[0].ver.major = 1;
    get_pkg_info[0].pkg_info[0].ver.minor = 3;
    get_pkg_info[0].pkg_info[0].ver.update = 35;
    get_pkg_info[0].pkg_info[0].ver.draft = 0;
    char *ice_pkg_name = (char *)malloc(ICE_PKG_NAME_SIZE*sizeof(char));
    memset(ice_pkg_name, 0, ICE_PKG_NAME_SIZE*sizeof(char));
    char ice_pkg_name_str[ICE_PKG_NAME_SIZE] = "ICE COMMS Package";
    for (int i = 0; i <= strlen(ice_pkg_name); i++)
    {
      ice_pkg_name[i] = ice_pkg_name_str[i];
    }
    // ice_pkg_name[i]
    memcpy(get_pkg_info[0].pkg_info[0].name, ice_pkg_name, sizeof(get_pkg_info[0].pkg_info[0].name));
    desc_complete_indir(0, get_pkg_info, sizeof(*get_pkg_info));
  } else if (d->opcode == ice_aqc_opc_set_rss_key) {
    struct ice_aqc_get_set_rss_key *v =
        reinterpret_cast<struct ice_aqc_get_set_rss_key *>(
                d->params.raw);
    
    desc_complete_indir(0, data, d->datalen);
  } else if (d->opcode == ice_aqc_opc_set_rss_lut) {
    struct ice_aqc_get_set_rss_lut *v =
        reinterpret_cast<struct ice_aqc_get_set_rss_lut *>(
                d->params.raw);
    desc_complete_indir(0, data, d->datalen);
  }
  else if (d->opcode == i40e_aqc_opc_set_switch_config) {
#ifdef DEBUG_ADMINQ
    cout <<  "  set switch config" << logger::endl;
#endif
    /* TODO: lots of interesting things here like l2 filtering etc. that are
     * relevant.
    struct i40e_aqc_set_switch_config *sc =
        reinterpret_cast<struct i40e_aqc_set_switch_config *>(
                d->params.raw);
    */
    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_add_vsi) {
#ifdef DEBUG_ADMINQ
    cout <<  "  get vsi parameters" << logger::endl;
#endif
    struct ice_aqc_add_update_free_vsi_resp *v =
        reinterpret_cast<struct ice_aqc_add_update_free_vsi_resp *>(
                d->params.raw);
    v->vsi_num = 1;
    struct ice_aqc_vsi_props pd;
    memset(&pd, 0, sizeof(pd));
    pd.valid_sections |=
        ICE_AQ_VSI_PROP_SW_VALID | ICE_AQ_VSI_PROP_RXQ_MAP_VALID |
        ICE_AQ_VSI_PROP_Q_OPT_VALID;
    desc_complete_indir(0, &pd, sizeof(pd));
  } else if (d->opcode == i40e_aqc_opc_update_vsi_parameters) {
#ifdef DEBUG_ADMINQ
    cout <<  "  update vsi parameters" << logger::endl;
#endif
    /* TODO */
    desc_complete(0);
  } else if (d->opcode == i40e_aqc_opc_set_dcb_parameters) {
#ifdef DEBUG_ADMINQ
    cout <<  "  set dcb parameters" << logger::endl;
#endif
    /* TODO */
    desc_complete(0);
  } else if (d->opcode == ice_aqc_opc_get_dflt_topo) {
#ifdef DEBUG_ADMINQ
    cout <<  "  configure vsi bw limit" << logger::endl;
#endif
    // cout<<"go to here"<<endl;
    struct ice_aqc_get_topo *get_topo = reinterpret_cast<struct ice_aqc_get_topo *>(
                d->params.raw);
    get_topo->port_num = 1;
    get_topo->num_branches = 1;
    // cout<<"size of topo_elem" << sizeof(dev.topo_elem)<<endl;
    // memset(topo_elem, 0, sizeof(*topo_elem));
    (dev.topo_elem).hdr.num_elems = 9;
    for (int i = 0; i < 9; i++)
    {
      dev.topo_elem.generic[i].node_teid = i;
    }
    dev.topo_elem.generic[0].parent_teid = 0xFFFFFFFF;
    dev.topo_elem.generic[0].data.elem_type = ICE_AQC_ELEM_TYPE_ROOT_PORT;
    
    dev.topo_elem.generic[1].parent_teid = 0;
    dev.topo_elem.generic[1].data.elem_type = ICE_AQC_ELEM_TYPE_TC;
    dev.topo_elem.generic[2].parent_teid = 1;
    dev.topo_elem.generic[2].data.elem_type = ICE_AQC_ELEM_TYPE_ENTRY_POINT;
    
    for (int i = 3; i < 9; i++)
    {
      dev.topo_elem.generic[i].parent_teid = 0;
      dev.topo_elem.generic[i].data.elem_type = ICE_AQC_ELEM_TYPE_TC;

    }

    desc_complete_indir(0, &dev.topo_elem, sizeof(dev.topo_elem));
  } else if (d->opcode == ice_aqc_opc_get_sched_elems) {
    // printf("l1");
    struct ice_aqc_sched_elem_cmd *get_elem_cmd = reinterpret_cast<ice_aqc_sched_elem_cmd *> (d->params.raw);
    get_elem_cmd->num_elem_resp = 1;
    // struct ice_aqc_get_elem *get_elem_buf = reinterpret_cast<ice_aqc_get_elem *> (data);
    struct ice_aqc_get_elem *get_elem = reinterpret_cast<struct ice_aqc_get_elem*> (data);
    // printf("l2");
    // printf("node id: %d", get_elem->generic[0].node_teid);
    // printf("l3");
    if (get_elem->generic[0].node_teid == 0){
      get_elem->generic[0].parent_teid = 0xFFFFFFFF;
      get_elem->generic[0].data.elem_type = ICE_AQC_ELEM_TYPE_ROOT_PORT;
    } else if (get_elem->generic[0].node_teid == 2) {
      get_elem->generic[0].parent_teid = 1;
      get_elem->generic[0].data.elem_type = ICE_AQC_ELEM_TYPE_ENTRY_POINT;
    } else {
      get_elem->generic[0].parent_teid = 0;
      get_elem->generic[0].data.elem_type = ICE_AQC_ELEM_TYPE_TC;
    }
    // printf("l4");
    desc_complete_indir(0, get_elem, sizeof(*get_elem));
    // printf("l5, data size: %d", sizeof(get_elem));
  } else if (d->opcode == ice_aqc_opc_add_sched_elems) {
    struct ice_aqc_sched_elem_cmd *get_elem_cmd = reinterpret_cast<ice_aqc_sched_elem_cmd *> (d->params.raw);
    get_elem_cmd->num_elem_resp = 1;
    struct ice_aqc_add_elem *add_elem = reinterpret_cast<struct ice_aqc_add_elem*> (data);
    // cout<< "1025 get heere1"<<endl;
    // cout<< "1025 get heere"<<endl;
    add_elem->generic[0].node_teid = add_elem->generic[0].parent_teid + 8;
    // get_elem.generic[0].data.elem_type = ICE_AQC_ELEM_TYPE_ENTRY_POINT;
    desc_complete_indir(0, add_elem, d->datalen);
  } else if (d->opcode == ice_aqc_opc_delete_sched_elems) {
    struct ice_aqc_sched_elem_cmd *delete_elem_cmd = reinterpret_cast<ice_aqc_sched_elem_cmd *> (d->params.raw);
    delete_elem_cmd->num_elem_resp = 1;
    struct ice_aqc_delete_elem *delete_elem = reinterpret_cast<struct ice_aqc_delete_elem*> (data);
    desc_complete_indir(0, data, d->datalen);
  } else if (d->opcode == ice_aqc_opc_query_sched_res){
    struct ice_aqc_sched_elem_cmd *query_elem = reinterpret_cast<ice_aqc_sched_elem_cmd *> (d->params.raw);
    query_elem->num_elem_resp = 1;
    struct ice_aqc_query_txsched_res_resp query_res;
    // = reinterpret_cast<struct ice_aqc_query_txsched_res_resp*> (data);
    query_res.sched_props.logical_levels = 7;
    query_res.sched_props.phys_levels = 2;
    query_res.layer_props[0].max_sibl_grp_sz = 7;
    
    for (int i = 1; i < 7; i++)
    {
      query_res.layer_props[i].max_sibl_grp_sz = 1024;
    }
    
    desc_complete_indir(0, &query_res, sizeof(query_res));
  } else if (d->opcode == ice_aqc_opc_add_txqs) {
    struct ice_aqc_add_txqs *add_txqs_cmd = reinterpret_cast<ice_aqc_add_txqs *> (d->params.raw);
    // add_txqs_cmd->num_qgrps = 1;
    struct ice_aqc_add_tx_qgrp *add_txqs = reinterpret_cast<ice_aqc_add_tx_qgrp *> (data);
    add_txqs->parent_teid = 2;
    add_txqs->num_txqs = 1;
    // add_txqs->txqs[0].txq_id = 0;
    add_txqs->txqs[0].q_teid = 10;
    add_txqs->txqs[0].info.elem_type = ICE_AQC_ELEM_TYPE_SE_GENERIC;
    add_txqs->txqs[0].info.cir_bw.bw_alloc = 100;
    add_txqs->txqs[0].info.eir_bw.bw_alloc = 100;
    if (add_txqs->txqs[0].txq_id >=4){
      cout<< "ice_aqc_opc_add_txqs error. txd id = "<< add_txqs->txqs[0].txq_id << logger::endl;
    }
    memcpy(dev.ctx_addr[add_txqs->txqs[0].txq_id], add_txqs->txqs[0].txq_ctx, sizeof(u8)*22);
    // dev.regs.qtx_ena[add_txqs->txqs[0].txq_id] = 1 ;
    // dev.lanmgr.qena_updated(add_txqs->txqs[0].txq_id, false);
    desc_complete_indir(0, data, d->datalen);
  }else if (d->opcode == ice_aqc_opc_dis_txqs) {
    struct ice_aqc_dis_txqs *dis_txqs_cmd = reinterpret_cast<ice_aqc_dis_txqs *> (d->params.raw);
    struct ice_aqc_dis_txq_item *dis_txqs = reinterpret_cast<ice_aqc_dis_txq_item *> (data);
    dis_txqs->num_qs = 1;
    desc_complete_indir(0, data, d->datalen);
  } else if (d->opcode == ice_aqc_opc_download_pkg) {
    struct ice_aqc_download_pkg *download_pkg = reinterpret_cast<ice_aqc_download_pkg *> (d->params.raw);
    struct ice_aqc_download_pkg_resp download_pkg_resp;
    // download_pkg_resp.error_info = ICE_AQ_RC_OK;
    desc_complete_indir(0, data, d->datalen);
  } else if (d->opcode == i40e_aqc_opc_query_vsi_bw_config) {
#ifdef DEBUG_ADMINQ
    cout <<  "  query vsi bw config" << logger::endl;
#endif
    struct i40e_aqc_query_vsi_bw_config_resp bwc;
    memset(&bwc, 0, sizeof(bwc));
    for (size_t i = 0; i < 8; i++)
      bwc.qs_handles[i] = 0xffff;
    desc_complete_indir(0, &bwc, sizeof(bwc));
  } else if (d->opcode == i40e_aqc_opc_query_vsi_ets_sla_config) {
#ifdef DEBUG_ADMINQ
    cout <<  "  query vsi ets sla config" << logger::endl;
#endif
    struct i40e_aqc_query_vsi_ets_sla_config_resp sla;
    memset(&sla, 0, sizeof(sla));
    for (size_t i = 0; i < 8; i++)
      sla.share_credits[i] = 127;
    desc_complete_indir(0, &sla, sizeof(sla));
  } else if (d->opcode == i40e_aqc_opc_remove_macvlan) {
#ifdef DEBUG_ADMINQ
    cout <<  "  remove macvlan" << logger::endl;
#endif
    struct i40e_aqc_macvlan *m =
        reinterpret_cast<struct i40e_aqc_macvlan *>(d->params.raw);
    struct i40e_aqc_remove_macvlan_element_data *rve =
        reinterpret_cast<struct i40e_aqc_remove_macvlan_element_data *>(data);
    for (uint16_t i = 0; i < m->num_addresses; i++)
      rve[i].error_code = I40E_AQC_REMOVE_MACVLAN_SUCCESS;

    desc_complete_indir(0, data, d->datalen);
  } else if (d->opcode == ice_aqc_opc_add_sw_rules) {
    struct ice_aqc_sw_rules *add_sw_rules_cmd = reinterpret_cast<ice_aqc_sw_rules *> (d->params.raw);
    struct ice_aqc_sw_rules_elem *add_sw_rules = reinterpret_cast<ice_aqc_sw_rules_elem*>(data);
    add_sw_rules->type = ICE_AQC_SW_RULES_T_LKUP_TX;
    add_sw_rules->pdata.lkup_tx_rx.src = 1;
    add_sw_rules->pdata.lkup_tx_rx.index = 0;
    desc_complete_indir(0, data, d->datalen);
  } else {
    cout <<  "  uknown opcode=" << d->opcode << logger::endl;
#ifdef DEBUG_ADMINQ
    cout <<  "  uknown opcode=" << d->opcode << logger::endl;
#endif
    // desc_complete(I40E_AQ_RC_ESRCH);
    desc_complete(0);
  }
}
}  // namespace i40e
