/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#include "logic.h"
//#include "scaling_factors_45nm_to_14nm.h"

#define debug 0

//selection_logic
selection_logic::selection_logic(
    bool   _is_default,
    int    win_entries_,
    int    issue_width_,
    const InputParameter *configure_interface,
    enum Device_ty device_ty_,
    enum Core_type core_ty_)
    //const ParseXML *_XML_interface)
 :is_default(_is_default),
  win_entries(win_entries_),
  issue_width(issue_width_),
  device_ty(device_ty_),
  core_ty(core_ty_)
 {
    	//uca_org_t result2;
    	l_ip=*configure_interface;
//		  cout << "selection_logic, techsize: " << l_ip.F_sz_um << ", wire: " << l_ip.wire_F_sz_um << ", finfet: " << l_ip.is_finfet << ", ncfet: " << l_ip.is_ncfet <<
//				  ", itrs: " << l_ip.is_itrs2012 << ", asap7: " << l_ip.is_asap7 << ", projection: " << l_ip.ic_proj_type << ", vdd: " << l_ip.vdd << endl;

    	local_result = init_interface(&l_ip);
    	//init_tech_params(l_ip.F_sz_um, false);
    	//win_entries=numIBEntries;//IQentries;
		//issue_width=issueWidth;
    	selection_power();
    	double sckRation = g_tp.sckt_co_eff;
    	power.readOp.dynamic *= sckRation;
    	power.writeOp.dynamic *= sckRation;
    	power.searchOp.dynamic *= sckRation;

    	double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);
    	power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;

    	double pg_reduction = power_gating_leakage_reduction(false);
    	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;

	 }

void selection_logic::selection_power()
{//based on cost effective superscalar processor TR pp27-31
  double Ctotal, Cor, Cpencode;
  int num_arbiter;
  double WSelORn, WSelORprequ, WSelPn, WSelPp, WSelEnn, WSelEnp;

  //TODO: the 0.8um process data is used.
  WSelORn    	=  	12.5 * l_ip.F_sz_um;//this was 10 micron for the 0.8 micron process
  WSelORprequ   = 	50 * l_ip.F_sz_um;//this was 40 micron for the 0.8 micron process
  WSelPn     	= 	12.5 * l_ip.F_sz_um;//this was 10mcron for the 0.8 micron process
  WSelPp     	=  	18.75 * l_ip.F_sz_um;//this was 15 micron for the 0.8 micron process
  WSelEnn    	=  	6.25 * l_ip.F_sz_um;//this was 5 micron for the 0.8 micron process
  WSelEnp    	=  	12.5 * l_ip.F_sz_um;//this was 10 micron for the 0.8 micron process

  Ctotal=0;
  num_arbiter=1;
  while(win_entries > 4)
    {
      win_entries = (int)ceil((double)win_entries / 4.0);
      num_arbiter += win_entries;
    }
  //the 4-input OR logic to generate anyreq
  Cor = 4 * drain_C_(WSelORn,NCH,1,1, g_tp.cell_h_def) + drain_C_(WSelORprequ,PCH,1,1, g_tp.cell_h_def);
  power.readOp.gate_leakage = cmos_Ig_leakage(WSelORn, WSelORprequ, 4, nor)*g_tp.peri_global.Vdd;

  //The total capacity of the 4-bit priority encoder
  Cpencode = drain_C_(WSelPn,NCH,1, 1, g_tp.cell_h_def) + drain_C_(WSelPp,PCH,1, 1, g_tp.cell_h_def) +
    2*drain_C_(WSelPn,NCH,1, 1, g_tp.cell_h_def) + drain_C_(WSelPp,PCH,2, 1, g_tp.cell_h_def) +
    3*drain_C_(WSelPn,NCH,1, 1, g_tp.cell_h_def) + drain_C_(WSelPp,PCH,3, 1, g_tp.cell_h_def) +
    4*drain_C_(WSelPn,NCH,1, 1, g_tp.cell_h_def) + drain_C_(WSelPp,PCH,4, 1, g_tp.cell_h_def) +//precompute priority logic
    2*4*gate_C(WSelEnn+WSelEnp,20.0)+
    4*drain_C_(WSelEnn,NCH,1, 1, g_tp.cell_h_def) + 2*4*drain_C_(WSelEnp,PCH,1, 1, g_tp.cell_h_def)+//enable logic
    (2*4+2*3+2*2+2)*gate_C(WSelPn+WSelPp,10.0);//requests signal

  Ctotal += issue_width * num_arbiter*(Cor+Cpencode);

  power.readOp.dynamic = Ctotal*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*2;//2 means the abitration signal need to travel round trip
  power.readOp.leakage = issue_width * num_arbiter *
      (cmos_Isub_leakage(WSelPn, WSelPp, 2, nor)/*approximate precompute with a nor gate*///grant1p
       + cmos_Isub_leakage(WSelPn, WSelPp, 3, nor)//grant2p
       + cmos_Isub_leakage(WSelPn, WSelPp, 4, nor)//grant3p
       + cmos_Isub_leakage(WSelEnn, WSelEnp, 2, nor)*4//enable logic
       + cmos_Isub_leakage(WSelEnn, WSelEnp, 1, inv)*2*3//for each grant there are two inverters, there are 3 grant sIsubnals
		  )*g_tp.peri_global.Vdd;
  power.readOp.gate_leakage = issue_width * num_arbiter *
      (cmos_Ig_leakage(WSelPn, WSelPp, 2, nor)/*approximate precompute with a nor gate*///grant1p
       + cmos_Ig_leakage(WSelPn, WSelPp, 3, nor)//grant2p
       + cmos_Ig_leakage(WSelPn, WSelPp, 4, nor)//grant3p
       + cmos_Ig_leakage(WSelEnn, WSelEnp, 2, nor)*4//enable logic
       + cmos_Ig_leakage(WSelEnn, WSelEnp, 1, inv)*2*3//for each grant there are two inverters, there are 3 grant signals
        )*g_tp.peri_global.Vdd;
}


dep_resource_conflict_check::dep_resource_conflict_check(
	const InputParameter *configure_interface,
	const CoreDynParam & dyn_p_,
	int   compare_bits_,
    bool   _is_default)
 :  l_ip(*configure_interface),
    coredynp(dyn_p_),
    compare_bits(compare_bits_),
	is_default(_is_default)
{
    	Wcompn    	=  	25 * l_ip.F_sz_um;//this was 20.0 micron for the 0.8 micron process
    	Wevalinvp   = 	25 * l_ip.F_sz_um;//this was 20.0 micron for the 0.8 micron process
    	Wevalinvn   = 	100 * l_ip.F_sz_um;//this was 80.0 mcron for the 0.8 micron process
    	Wcomppreequ =  	50 * l_ip.F_sz_um;//this was 40.0  micron for the 0.8 micron process
    	WNORn    	=  	6.75 * l_ip.F_sz_um;//this was 5.4 micron for the 0.8 micron process
    	WNORp    	=  	38.125 * l_ip.F_sz_um;//this was 30.5 micron for the 0.8 micron process
//  	  cout << "dep_resource_conflict_check, techsize: " << l_ip.F_sz_um << ", wire: " << l_ip.wire_F_sz_um << ", finfet: " << l_ip.is_finfet << ", ncfet: " << l_ip.is_ncfet <<
  //			  ", itrs: " << l_ip.is_itrs2012 << ", asap7: " << l_ip.is_asap7 << ", projection: " << l_ip.ic_proj_type << ", vdd: " << l_ip.vdd << endl;

    	local_result = init_interface(&l_ip);

    	if (coredynp.core_ty==Inorder)
   		    compare_bits += 16 + 8 + 8;//TODO: opcode bits + log(shared resources) + REG TAG BITS-->opcode comparator
    	else
    		compare_bits += 16 + 8 + 8;

   		conflict_check_power();
    	double sckRation = g_tp.sckt_co_eff;
    	power.readOp.dynamic *= sckRation;
    	power.writeOp.dynamic *= sckRation;
    	power.searchOp.dynamic *= sckRation;

}

void dep_resource_conflict_check::conflict_check_power()
{
	double Ctotal;
	int num_comparators;
	num_comparators = 3*((coredynp.decodeW) * (coredynp.decodeW)-coredynp.decodeW);//2(N*N-N) is used for source to dest comparison, (N*N-N) is used for dest to dest comparision.
	//When decode-width ==1, no dcl logic

	Ctotal = num_comparators * compare_cap();
	//printf("%i,%s\n",XML_interface->sys.core[0].predictor.predictor_entries,XML_interface->sys.core[0].predictor.prediction_scheme);

	power.readOp.dynamic=Ctotal*/*CLOCKRATE*/g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/*AF*/;
	power.readOp.leakage=num_comparators*compare_bits*2*simplified_nmos_leakage(Wcompn,  false);

	double long_channel_device_reduction = longer_channel_device_reduction(Core_device, coredynp.core_ty);
	power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;
	power.readOp.gate_leakage=num_comparators*compare_bits*2*cmos_Ig_leakage(Wcompn, 0, 2, nmos);

	double pg_reduction = power_gating_leakage_reduction(false);
	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;


}

/* estimate comparator power consumption (this comparator is similar
   to the tag-match structure in a CAM */
double dep_resource_conflict_check::compare_cap()
{
  double c1, c2;

  WNORp = WNORp * compare_bits/2.0;//resize the big NOR gate at the DCL according to fan in.
  /* bottom part of comparator */
  c2 = (compare_bits)*(drain_C_(Wcompn,NCH,1,1, g_tp.cell_h_def)+drain_C_(Wcompn,NCH,2,1, g_tp.cell_h_def))+
  drain_C_(Wevalinvp,PCH,1,1, g_tp.cell_h_def) + drain_C_(Wevalinvn,NCH,1,1, g_tp.cell_h_def);

  /* top part of comparator */
  c1 = (compare_bits)*(drain_C_(Wcompn,NCH,1,1, g_tp.cell_h_def)+drain_C_(Wcompn,NCH,2,1, g_tp.cell_h_def)+
		  drain_C_(Wcomppreequ,NCH,1,1, g_tp.cell_h_def)) +  gate_C(WNORn + WNORp,10.0) +
		  drain_C_(WNORp,NCH,2,1, g_tp.cell_h_def) + compare_bits*drain_C_(WNORn,NCH,2,1, g_tp.cell_h_def);
  return(c1 + c2);

}

void dep_resource_conflict_check::leakage_feedback(double temperature)
{
//  cout << "dep_resource_conflict_check leakage_feedback(), techsize: " << l_ip.F_sz_um << ", wire: " << l_ip.wire_F_sz_um << ", finfet: " << l_ip.is_finfet << ", ncfet: " << l_ip.is_ncfet <<
//			  ", itrs: " << l_ip.is_itrs2012 << ", asap7: " << l_ip.is_asap7 << ", projection: " << l_ip.ic_proj_type << ", vdd: " << l_ip.vdd << endl;
  l_ip.temp = (unsigned int)round(temperature/10.0)*10;
  uca_org_t init_result = init_interface(&l_ip); // init_result is dummy

  // This is part of conflict_check_power()
  int num_comparators = 3*((coredynp.decodeW) * (coredynp.decodeW)-coredynp.decodeW);//2(N*N-N) is used for source to dest comparison, (N*N-N) is used for dest to dest comparision.
  power.readOp.leakage=num_comparators*compare_bits*2*simplified_nmos_leakage(Wcompn,  false);

  double long_channel_device_reduction = longer_channel_device_reduction(Core_device, coredynp.core_ty);
  power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;
  power.readOp.gate_leakage=num_comparators*compare_bits*2*cmos_Ig_leakage(Wcompn, 0, 2, nmos);

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;

}

//TODO: add inverter and transmission gate base DFF.

DFFCell::DFFCell(
		bool _is_dram,
		double _WdecNANDn,
		double _WdecNANDp,
		double _cell_load,
		const InputParameter *configure_interface)
:is_dram(_is_dram),
cell_load(_cell_load),
WdecNANDn(_WdecNANDn),
WdecNANDp(_WdecNANDp)
{//this model is based on the NAND2 based DFF.
			l_ip=*configure_interface;
//			area.set_area(730*l_ip.F_sz_um*l_ip.F_sz_um);
			area.set_area(5*compute_gate_area(NAND, 2,WdecNANDn,WdecNANDp, g_tp.cell_h_def)
				+ compute_gate_area(NAND, 2,WdecNANDn,WdecNANDn, g_tp.cell_h_def));
}


double DFFCell::fpfp_node_cap(unsigned int fan_in, unsigned int fan_out)
{
  double Ctotal = 0;
  //printf("WdecNANDn = %E\n", WdecNANDn);

  /* part 1: drain cap of NAND gate */
  Ctotal += drain_C_(WdecNANDn, NCH, 2, 1, g_tp.cell_h_def, is_dram) + fan_in * drain_C_(WdecNANDp, PCH, 1, 1, g_tp.cell_h_def, is_dram);

  /* part 2: gate cap of NAND gates */
  Ctotal += fan_out * gate_C(WdecNANDn + WdecNANDp, 0, is_dram);

  return Ctotal;
}


void DFFCell::compute_DFF_cell()
{
	double c1, c2, c3, c4, c5, c6;
	   /* node 5 and node 6 are identical to node 1 in capacitance */
	   c1 = c5 = c6 = fpfp_node_cap(2, 1);
	   c2 = fpfp_node_cap(2, 3);
	   c3 = fpfp_node_cap(3, 2);
	   c4 = fpfp_node_cap(2, 2);

	   //cap-load of the clock signal in each Dff, actually the clock signal only connected to one NAND2
	   clock_cap= 2 * gate_C(WdecNANDn + WdecNANDp, 0, is_dram);
	   e_switch.readOp.dynamic += (c4 + c1 + c2 + c3 + c5 + c6 + 2*cell_load)*0.5*g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;;

	   /* no 1/2 for e_keep and e_clock because clock signal switches twice in one cycle */
	   e_keep_1.readOp.dynamic += c3 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd ;
	   e_keep_0.readOp.dynamic += c2 * g_tp.peri_global.Vdd * g_tp.peri_global.Vdd ;
	   e_clock.readOp.dynamic += clock_cap* g_tp.peri_global.Vdd * g_tp.peri_global.Vdd;;

	   /* static power */
	   e_switch.readOp.leakage +=  (cmos_Isub_leakage(WdecNANDn, WdecNANDp, 2, nand)*5//5 NAND2 and 1 NAND3 in a DFF
                                           + cmos_Isub_leakage(WdecNANDn, WdecNANDn, 3, nand))*g_tp.peri_global.Vdd;
	   e_switch.readOp.gate_leakage += (cmos_Ig_leakage(WdecNANDn, WdecNANDp, 2, nand)*5//5 NAND2 and 1 NAND3 in a DFF
                                           + cmos_Ig_leakage(WdecNANDn, WdecNANDn, 3, nand))*g_tp.peri_global.Vdd;
	   //printf("leakage =%E\n",cmos_Ileak(1, is_dram) );
}

Pipeline::Pipeline(
		const InputParameter *configure_interface,
		const CoreDynParam & dyn_p_,
		enum Device_ty device_ty_,
		bool _is_core_pipeline,
		bool _is_default)
: l_ip(*configure_interface),
  coredynp(dyn_p_),
  device_ty(device_ty_),
  is_core_pipeline(_is_core_pipeline),
  is_default(_is_default),
  num_piperegs(0.0)
{
//	  cout << "Pipeline, techsize: " << l_ip.F_sz_um << ", wire: " << l_ip.wire_F_sz_um << ", finfet: " << l_ip.is_finfet << ", ncfet: " << l_ip.is_ncfet <<
//			  ", itrs: " << l_ip.is_itrs2012 << ", asap7: " << l_ip.is_asap7 << ", projection: " << l_ip.ic_proj_type << ", vdd: " << l_ip.vdd << endl;

	local_result = init_interface(&l_ip);
	if (!coredynp.Embedded)
		process_ind = true;
	else
		process_ind = false;
	WNANDn = (process_ind)? 25 *   l_ip.F_sz_um : g_tp.min_w_nmos_ ;//this was  20 micron for the 0.8 micron process
	WNANDp = (process_ind)? 37.5 * l_ip.F_sz_um : g_tp.min_w_nmos_*pmos_to_nmos_sz_ratio();//this was  30 micron for the 0.8 micron process
	load_per_pipeline_stage = 2*gate_C(WNANDn + WNANDp, 0, false);
	compute();
}

void Pipeline::compute()
{
	compute_stage_vector();
	DFFCell pipe_reg(false, WNANDn,WNANDp, load_per_pipeline_stage, &l_ip);
	pipe_reg.compute_DFF_cell();

	double clock_power_pipereg = num_piperegs * pipe_reg.e_clock.readOp.dynamic;
	//******************pipeline power: currently, we average all the possibilities of the states of DFFs in the pipeline. A better way to do it is to consider
	//the harming distance of two consecutive signals, However McPAT does not have plan to do this in near future as it focuses on worst case power.
	double pipe_reg_power = num_piperegs * (pipe_reg.e_switch.readOp.dynamic+pipe_reg.e_keep_0.readOp.dynamic+pipe_reg.e_keep_1.readOp.dynamic)/3+clock_power_pipereg;
	double pipe_reg_leakage = num_piperegs * pipe_reg.e_switch.readOp.leakage;
	double pipe_reg_gate_leakage = num_piperegs * pipe_reg.e_switch.readOp.gate_leakage;
	power.readOp.dynamic	+=pipe_reg_power;
	power.readOp.leakage	+=pipe_reg_leakage;
	power.readOp.gate_leakage	+=pipe_reg_gate_leakage;
	area.set_area(num_piperegs * pipe_reg.area.get_area());

	double long_channel_device_reduction = longer_channel_device_reduction(device_ty, coredynp.core_ty);
	power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;

	double pg_reduction = power_gating_leakage_reduction(false);
	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;

	double sckRation = g_tp.sckt_co_eff;
	power.readOp.dynamic *= sckRation;
	power.writeOp.dynamic *= sckRation;
	power.searchOp.dynamic *= sckRation;
	double macro_layout_overhead = g_tp.macro_layout_overhead;
	if (!coredynp.Embedded)
		area.set_area(area.get_area()*macro_layout_overhead);
}

void Pipeline::compute_stage_vector()
{
	double num_stages, tot_stage_vector, per_stage_vector;
	int opcode_length = coredynp.x86? coredynp.micro_opcode_length:coredynp.opcode_length;
	//Hthread = thread_clock_gated? 1:num_thread;

  if (!is_core_pipeline)
  {
	num_piperegs=l_ip.pipeline_stages*l_ip.per_stage_vector;//The number of pipeline stages are calculated based on the achievable throughput and required throughput
  }
  else
  {
	if (coredynp.core_ty==Inorder)
	{
		/* assume 6 pipe stages and try to estimate bits per pipe stage */
		/* pipe stage 0/IF */
		num_piperegs += coredynp.pc_width*2*coredynp.num_hthreads;
		/* pipe stage IF/ID */
		num_piperegs += coredynp.fetchW*(coredynp.instruction_length + coredynp.pc_width)*coredynp.num_hthreads;
		/* pipe stage IF/ThreadSEL */
		if (coredynp.multithreaded) num_piperegs += coredynp.num_hthreads*coredynp.perThreadState; //8 bit thread states
		/* pipe stage ID/EXE */
		num_piperegs += coredynp.decodeW*(coredynp.instruction_length + coredynp.pc_width + pow(2.0,opcode_length)+ 2*coredynp.int_data_width)*coredynp.num_hthreads;
		/* pipe stage EXE/MEM */
		num_piperegs += coredynp.issueW*(3 * coredynp.arch_ireg_width + pow(2.0,opcode_length) + 8*2*coredynp.int_data_width/*+2*powers (2,reg_length)*/);
		/* pipe stage MEM/WB the 2^opcode_length means the total decoded signal for the opcode*/
		num_piperegs += coredynp.issueW*(2*coredynp.int_data_width + pow(2.0,opcode_length) + 8*2*coredynp.int_data_width/*+2*powers (2,reg_length)*/);
//		/* pipe stage 5/6 */
//		num_piperegs += issueWidth*(data_width + powers (2,opcode_length)/*+2*powers (2,reg_length)*/);
//		/* pipe stage 6/7 */
//		num_piperegs += issueWidth*(data_width + powers (2,opcode_length)/*+2*powers (2,reg_length)*/);
//		/* pipe stage 7/8 */
//		num_piperegs += issueWidth*(data_width + powers (2,opcode_length)/**2*powers (2,reg_length)*/);
//		/* assume 50% extra in control signals (rule of thumb) */
		num_stages=6;

	}
	else
	{
		/* assume 12 stage pipe stages and try to estimate bits per pipe stage */
		/*OOO: Fetch, decode, rename, IssueQ, dispatch, regread, EXE, MEM, WB, CM */

		/* pipe stage 0/1F*/
		num_piperegs += coredynp.pc_width*2*coredynp.num_hthreads ;//PC and Next PC
		/* pipe stage IF/ID */
		num_piperegs += coredynp.fetchW*(coredynp.instruction_length + coredynp.pc_width)*coredynp.num_hthreads;//PC is used to feed branch predictor in ID
		/* pipe stage 1D/Renaming*/
		num_piperegs += coredynp.decodeW*(coredynp.instruction_length + coredynp.pc_width)*coredynp.num_hthreads;//PC is for branch exe in later stage.
		/* pipe stage Renaming/wire_drive */
		num_piperegs += coredynp.decodeW*(coredynp.instruction_length + coredynp.pc_width);
		/* pipe stage Renaming/IssueQ */
		num_piperegs += coredynp.issueW*(coredynp.instruction_length  + coredynp.pc_width + 3*coredynp.phy_ireg_width)*coredynp.num_hthreads;//3*coredynp.phy_ireg_width means 2 sources and 1 dest
		/* pipe stage IssueQ/Dispatch */
		num_piperegs += coredynp.issueW*(coredynp.instruction_length + 3 * coredynp.phy_ireg_width);
		/* pipe stage Dispatch/EXE */

		num_piperegs += coredynp.issueW*(3 * coredynp.phy_ireg_width + coredynp.pc_width + pow(2.0,opcode_length)/*+2*powers (2,reg_length)*/);
		/* 2^opcode_length means the total decoded signal for the opcode*/
		num_piperegs += coredynp.issueW*(2*coredynp.int_data_width + pow(2.0,opcode_length)/*+2*powers (2,reg_length)*/);
		/*2 source operands in EXE; Assume 2EXE stages* since we do not really distinguish OP*/
		num_piperegs += coredynp.issueW*(2*coredynp.int_data_width + pow(2.0,opcode_length)/*+2*powers (2,reg_length)*/);
		/* pipe stage EXE/MEM, data need to be read/write, address*/
		num_piperegs += coredynp.issueW*(coredynp.int_data_width + coredynp.v_address_width + pow(2.0,opcode_length)/*+2*powers (2,reg_length)*/);//memory Opcode still need to be passed
		/* pipe stage MEM/WB; result data, writeback regs */
		num_piperegs += coredynp.issueW*(coredynp.int_data_width + coredynp.phy_ireg_width /* powers (2,opcode_length) + (2,opcode_length)+2*powers (2,reg_length)*/);
		/* pipe stage WB/CM ; result data, regs need to be updated, address for resolve memory ops in ROB's top*/
		num_piperegs += coredynp.commitW*(coredynp.int_data_width + coredynp.v_address_width + coredynp.phy_ireg_width/*+ powers (2,opcode_length)*2*powers (2,reg_length)*/)*coredynp.num_hthreads;
//		if (multithreaded)
//		{
//
//		}
		num_stages=12;

	}

	/* assume 50% extra in control registers and interrupt registers (rule of thumb) */
	num_piperegs = num_piperegs * 1.5;
	tot_stage_vector=num_piperegs;
	per_stage_vector=tot_stage_vector/num_stages;

	if (coredynp.core_ty==Inorder)
	{
		if (coredynp.pipeline_stages>6)
			num_piperegs= per_stage_vector*coredynp.pipeline_stages;
	}
	else//OOO
	{
		if (coredynp.pipeline_stages>12)
			num_piperegs= per_stage_vector*coredynp.pipeline_stages;
	}
  }
//  cout << "num_pipeline regs: " << num_piperegs << endl;
}

FunctionalUnit::FunctionalUnit(ParseXML *XML_interface, int ithCore_, InputParameter* interface_ip_,const CoreDynParam & dyn_p_, enum FU_type fu_type_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 fu_type(fu_type_)
{
    double area_t;//, leakage, gate_leakage;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
//    double pmos_to_nmos_sizing_r = 2.41; //45-nm cmos
//	cout << "FunctionalUnit:: pmos_to_nmos_sizing_r= " << pmos_to_nmos_sizing_r << endl;
	clockRate = coredynp.clockRate;
	executionTime = coredynp.executionTime;
//	cout << "FunctionalUnit:: clockRate = " << clockRate << ", executionTime = " << executionTime << endl;

	//XML_interface=_XML_interface;
	uca_org_t result2;
	  if(debug)
	  cout << "FunctionalU, techsize: " << interface_ip.F_sz_um << ", wire: " << interface_ip.wire_F_sz_um << ", finfet: " << interface_ip.is_finfet << ", ncfet: " << interface_ip.is_ncfet <<
			  ", itrs: " << interface_ip.is_itrs2012 << ", asap7: " << interface_ip.is_asap7 << ", projection: " << interface_ip.ic_proj_type <<
			  ", vdd: " << interface_ip.vdd << endl;

//	result2 = init_interface(&interface_ip);
	if (XML->sys.Embedded)
	{
/*		if (fu_type == FPU)
		{
			num_fu=coredynp.num_fpus;
			area_t = 4.47*1e6*(g_ip->F_sz_nm*g_ip->F_sz_nm/90.0/90.0);//this is um^2 The base number
			  if(debug)
			cout << "FPU Area: " << area_t <<", tech_size: " << g_ip->F_sz_nm << endl;
			//4.47 contains both VFP and NEON processing unit, VFP is about 40% and NEON is about 60%
			if (g_ip->F_sz_nm>90)
				area_t = 4.47*1e6*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2
			leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
			gate_leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W

			base_energy = 0;
			per_access_energy = 1.15/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per Hz energy(nJ)
			//FPU power from Sandia's processor sizing tech report
			FU_height=(18667*num_fu)*interface_ip.F_sz_um;//FPU from Sun's data
		}
		else if (fu_type == ALU)
		{
			num_fu=coredynp.num_alus;
			area_t = 280*260*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
			  if(debug)
			cout << "ALU Area: " << area_t <<", tech_size: " << g_ip->F_sz_nm<< endl;
			leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
			gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;

			base_energy = 0;
			per_access_energy = 1.15/3/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per cycle energy(nJ)
			FU_height=(6222*num_fu)*interface_ip.F_sz_um;//integer ALU

		}
		else if (fu_type == MUL)
		{
			num_fu=coredynp.num_muls;
			area_t = 280*260*3*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
			  if(debug)
			cout << "MUL Area: " << area_t <<", tech_size: " << g_ip->F_sz_nm<< endl;
			leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
			gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;

			base_energy = 0;
			per_access_energy = 1.15*2/3/1e9/4/1.3/1.3*g_tp.peri_global.Vdd*g_tp.peri_global.Vdd*(g_ip->F_sz_nm/90.0);//(g_tp.peri_global.Vdd*g_tp.peri_global.Vdd/1.2/1.2);//0.00649*1e-9; //This is per cycle energy(nJ), coefficient based on Wattch
			FU_height=(9334*num_fu )*interface_ip.F_sz_um;//divider/mul from Sun's data
		}
		else
		{
			cout<<"Unknown Functional Unit Type"<<endl;
			exit(0);
		}
		per_access_energy *=0.5;//According to ARM data embedded processor has much lower per acc energy
*/
	}
	else
	{
	   int voltage_case = -1;

		if(interface_ip.vdd == 0.3)
			voltage_case = 0;
		else if(interface_ip.vdd == 0.4)
			voltage_case = 1;
		else if(interface_ip.vdd == 0.5)
			voltage_case = 2;
		else if(interface_ip.vdd == 0.6)
			voltage_case = 3;
		else if(interface_ip.vdd == 0.7)
			voltage_case = 4;
		else
			voltage_case = -1;

		if (fu_type == FPU)
		{
			num_fu=coredynp.num_fpus;

			//divya applied FinFET 0.7V power numbers provided 27-3-2022
			area_t = 18169; //um2
			base_energy = 0;

			if(interface_ip.is_ncfet == 0)	//FinFET
			{
				switch(voltage_case) {
					case 0:
						per_access_energy = 3 * 1.026*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 39.64*1e-6; //W
					break;
					case 1:
						per_access_energy = 3 * 2.031*1e-12; //(J) FIXed using NCFET Edyn_per_access(0.4V)
						leakage = 66.82*1e-6; //W
					break;
					case 2:
						per_access_energy = 3 * 3.006*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 105.02*1e-6; //W
					break;
					case 3:
						per_access_energy = 3 * 4.352*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 157.72*1e-6; //W
					break;
					case 4:
						per_access_energy = 3 * 6.17*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 231.66*1e-6; //W
					break;
					default:
						cout << "Error! Check vdd" << endl;
				}
			}
			else	//NCFET
			{
				switch(voltage_case) {
					case 0:
						per_access_energy = 3 * 1.37*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 25.85*1e-6; //W
					break;
					case 1:
						per_access_energy = 3 * 2.496*1e-12; //(J)
						leakage = 29.13*1e-6; //W
					break;
					case 2:
						per_access_energy = 3 * 3.900*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 31.82*1e-6; //W
					break;
					case 3:
						per_access_energy = 3 * 5.645*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 30.44*1e-6; //W
					break;
					case 4:
						per_access_energy = 3 * 6.62*1e-12; //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 29.88*1e-6; //W
					break;
					default:
						cout << "Error! Check vdd" << endl;
				}
			}

			gate_leakage = 0;
			FU_height = sqrt(area_t*num_fu);	//<- optimized height (um). //sqrt(area)*num_fu=135*num_fu; //um obtained from BOOM

			if(debug)
			{
			cout << "FPU num_fu = " << num_fu << ", area_t = " << area_t << ", leakage = " << leakage
				<< ", base_energy(J) = " << base_energy << ", per_access_energy (nJ) = " << per_access_energy * 1e+9  << endl;
			cout << "FPU FU_height: " << FU_height <<", tech_size: " << interface_ip.F_sz_um<< endl;
			}
		}
		else if (fu_type == ALU)
		{
			num_fu=coredynp.num_alus;

			//divya applied FinFET 0.7V power numbers provided 27-3-2022
			area_t = 3775; //ALU1+alu2+ifpu	//8862; //um2
			base_energy = 0;
			if(interface_ip.is_ncfet == 0)	//FinFET
			{
				switch(voltage_case) {
					case 0:
						per_access_energy = 1 * 2.184*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 9.12*1e-6; //W
					break;
					case 1:
						per_access_energy = 1 * 4.361*1e-12; //(J) FIXed using NCFET Edyn_per_access(0.4V)
						leakage = 15.37*1e-6; //W
					break;
					case 2:
						per_access_energy = 1 * 6.359*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 24.16*1e-6; //W
					break;
					case 3:
						per_access_energy = 1 * 9.196*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 36.29*1e-6; //W
					break;
					case 4:
						per_access_energy = 1 * 13.11*1e-12; //(J) dynamic_power*data_arrival_time //considered 1 cycles for ALU operation, so multiplied by 1
						leakage = 53.73*1e-6; //ALU1+alu2+ifpu
					break;
					default:
						cout << "Error! Check vdd" << endl;
				}
			}
			else	//NCFET
			{
				switch(voltage_case) {
					case 0:
						per_access_energy = 1 * 2.902*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 5.948*1e-6; //W
					break;
					case 1:
						per_access_energy = 1 * 5.285*1e-12; //(J)
						leakage = 6.703*1e-6; //W
					break;
					case 2:
						per_access_energy = 1 * 8.254*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 7.322*1e-6; //W
					break;
					case 3:
						per_access_energy = 1 * 11.944*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 7.004*1e-6; //W
					break;
					case 4:
						per_access_energy = 1 * 14.121*1e-12; //(J) dynamic_power*data_arrival_time //considered 1 cycles for ALU operation, so multiplied by 1
						leakage = 6.931*1e-6; //W
					break;
					default:
						cout << "Error! Check vdd" << endl;
				}
			}
			gate_leakage = 0;
			FU_height= sqrt(area_t*num_fu); //<- optimized height (um). //sqrt(area)*num_fu=94*num_fu; //um obtained from BOOM

			if(debug)
			{
			cout << "ALU num_fu = " << num_fu << ", area_t = " << area_t << ", leakage = " << leakage
				<< ", base_energy(J) = " << base_energy << ", per_access_energy (nJ) = " << per_access_energy * 1e+9 << endl;
			cout << "ALU FU_height: " << FU_height <<", tech_size: " << interface_ip.F_sz_um<< endl;
			}
		}
		else if (fu_type == MUL)
		{
			num_fu=coredynp.num_muls;

			//divya applied FinFET 0.7V power numbers provided 27-3-2022
			area_t = 1270 + 4931; //MUL+DIV	//2004; //um2
			base_energy = 0;
			if(interface_ip.is_ncfet == 0)	//FinFET
			{
				switch(voltage_case) {
					case 0:
						per_access_energy = 3 * 0.31*1e-12 + 10 * 0.12*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 8.616*1e-6 + 3.052*1e-6; //W
					break;
					case 1:
						per_access_energy = 3 * 0.59*1e-12 + 10 * 0.224*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 14.521*1e-6 + 5.144*1e-6; //W
					break;
					case 2:
						per_access_energy = 3 * 0.894*1e-12 + 10 * 0.349*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 22.824*1e-6 + 8.085*1e-6; //W
					break;
					case 3:
						per_access_energy = 3 * 1.293*1e-12 + 10 * 0.506*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 34.278*1e-6 + 12.142*1e-6; //W
					break;
					case 4:
						per_access_energy = 3 * 1.84*1e-12 + 10 * 0.71*1e-12; // 3 cycles for MUL, 10 cycles for DIV// (J)dynamic_power*data_arrival_time //considered 3 cycles for MUL operation, so multiplied by 3
						leakage = 48.48*1e-6 + 17.93*1e-6; // MUL+DIV //W
					break;
					default:
						cout << "Error! Check vdd" << endl;
				}

			}
			else	//NCFET
			{
				switch(voltage_case) {
					case 0:
						per_access_energy = 3 * 0.406*1e-12 + 10 * 0.161*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 5.618*1e-6 + 1.990*1e-6; //W
					break;
					case 1:
						per_access_energy = 3 * 0.739*1e-12 + 10 * 0.239*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 6.331*1e-6 + 2.243*1e-6; //W
					break;
					case 2:
						per_access_energy = 3 * 1.154*1e-12 + 10 * 0.459*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 6.916*1e-6 + 2.450*1e-6; //W
					break;
					case 3:
						per_access_energy = 3 * 1.67*1e-12 + 10 * 0.665*1e-12; //(J) dynamic_power*data_arrival_time
						leakage = 6.616*1e-6 + 2.343*1e-6; //W
					break;
					case 4:
						per_access_energy = 3 * 1.97*1e-12 + 10 * 0.77*1e-12; // 3 cycles for MUL, 10 cycles for DIV //(J) dynamic_power*data_arrival_time //considered 3 cycles for FPU operation, so multiplied by 3
						leakage = 6.25*1e-6 + 2.31*1e-6; // MUL+DIV //W
					break;
					default:
						cout << "Error! Check vdd" << endl;
				}
			}
			gate_leakage = 0;
			FU_height= sqrt(area_t*num_fu);	//<- optimized height (um). //sqrt(area)*num_fu=45*num_fu; //um obtained from BOOM

			if(debug)
			{
			cout << "MUL num_fu = " << num_fu << ", area_t = " << area_t << ", leakage = " << leakage
				<< ", base_energy(J) = " << base_energy << ", per_access_energy (nJ) = " << per_access_energy * 1e+9  << endl;
			cout << "MUL FU_height: " << FU_height <<", tech_size: " << interface_ip.F_sz_um<< endl;
			}
		}

		else
		{
			cout<<"Unknown Functional Unit Type"<<endl;
			exit(0);
		}
	}
    area.set_area(area_t*num_fu);
    if(debug)
    cout << "area: " << area.get_area() << endl;
    if(debug)
    cout << "logic.cc:: FPU-0, ALU-1, MUL-2: " << fu_type << ", num_fu: " << num_fu << ", leakage: " << leakage << endl;

    leakage *= num_fu;

    if(debug)
    cout << "logic.cc:: FPU-0, ALU-1, MUL-2: " << fu_type << ", numfu_leakage: " << leakage << endl;

    gate_leakage *=num_fu;
	double macro_layout_overhead = g_tp.macro_layout_overhead;

	area.set_area(area.get_area()*macro_layout_overhead);
 //   cout << "area with overhead: " << area.get_area() << endl;
}

void FunctionalUnit::computeEnergy(bool is_tdp)
{
	double pppm_t[4]    = {1,1,1,1};
	double FU_duty_cycle;
	if (is_tdp)
	{
		set_pppm(pppm_t, 2, 2, 2, 2);//2 means two source operands needs to be passed for each int instruction.
		if (fu_type == FPU)
		{
			stats_t.readAc.access = num_fu;
			tdp_stats = stats_t;
			FU_duty_cycle = coredynp.FPU_duty_cycle;
		}
		else if (fu_type == ALU)
		{
			stats_t.readAc.access = 1*num_fu;
			tdp_stats = stats_t;
			FU_duty_cycle = coredynp.ALU_duty_cycle;
		}
		else if (fu_type == MUL)
		{
			stats_t.readAc.access = num_fu;
			tdp_stats = stats_t;
			FU_duty_cycle = coredynp.MUL_duty_cycle;
		}

	    //power.readOp.dynamic = base_energy/clockRate + energy*stats_t.readAc.access;
	    power.readOp.dynamic = per_access_energy*stats_t.readAc.access + base_energy/clockRate;
		double sckRation = g_tp.sckt_co_eff;
		power.readOp.dynamic *= sckRation*FU_duty_cycle;
		power.writeOp.dynamic *= sckRation;
		power.searchOp.dynamic *= sckRation;

	    power.readOp.leakage = leakage;
	    power.readOp.gate_leakage = gate_leakage;
	    double long_channel_device_reduction = longer_channel_device_reduction(Core_device, coredynp.core_ty);
	    power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;

    	double pg_reduction = power_gating_leakage_reduction(false);
    	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;
	}
	else
	{
		if (fu_type == FPU)
		{
			stats_t.readAc.access = XML->sys.core[ithCore].fpu_accesses;
			rtp_stats = stats_t;
		}
		else if (fu_type == ALU)
		{
			stats_t.readAc.access = XML->sys.core[ithCore].ialu_accesses;
			rtp_stats = stats_t;
		}
		else if (fu_type == MUL)
		{
			stats_t.readAc.access = XML->sys.core[ithCore].mul_accesses;
			rtp_stats = stats_t;
		}

	    //rt_power.readOp.dynamic = base_energy*executionTime + energy*stats_t.readAc.access;
	    rt_power.readOp.dynamic = per_access_energy*stats_t.readAc.access + base_energy*executionTime;

	    if(debug) {
	    	cout << fu_type << "is_tdp : " << is_tdp << ", exec time (sec) : "<< executionTime  << ", per access energy (J) :" << per_access_energy << ", #accesses : " << stats_t.readAc.access
	    		<< " access energy (J) : " << per_access_energy*stats_t.readAc.access << ", access power (W) : " << per_access_energy*stats_t.readAc.access/executionTime
	    		<< ", base power (W): " <<base_energy <<  ", base energy (J): " << base_energy*executionTime << endl;
	    	cout << "total dynamic energy (J) : " << rt_power.readOp.dynamic << ", total dynamic power (W) : " << rt_power.readOp.dynamic/executionTime << endl;
	    	cout << "total leakage power (W) : " << power.readOp.leakage << ", total leakage energy (J) : " << power.readOp.leakage*executionTime << endl;
	    }

		double sckRation = g_tp.sckt_co_eff;
		rt_power.readOp.dynamic *= sckRation;
		rt_power.writeOp.dynamic *= sckRation;
		rt_power.searchOp.dynamic *= sckRation;
	}
}

void FunctionalUnit::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;

//	cout << indent_str_next << "Results Broadcast Bus Area = " << bypass->area.get_area() *1e-6 << " mm^2" << endl;
	if (is_tdp)
	{
		if (fu_type == FPU)
		{
			cout << indent_str << "Floating Point Units (FPUs) (Count: "<< coredynp.num_fpus <<" ):" << endl;
			cout << indent_str_next << "Area = " << area.get_area()*1e-6  << " mm^2" << endl;
			cout << indent_str_next << "per_access_energy = " << per_access_energy * 1e+12 << " pJ" << endl;
//			cout << indent_str_next << "base_energy = " << base_energy * 1e+9 << " nJ" << endl;
			cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage  << " W" << endl;
			cout << indent_str_next<< "Subthreshold Leakage = "
						<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
							<< (power.readOp.power_gated_leakage * (long_channel? power.readOp.longer_channel_leakage/power.readOp.leakage:1) )  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
		}
		else if (fu_type == ALU)
		{
			cout << indent_str << "Integer ALUs (Count: "<< coredynp.num_alus <<" ):" << endl;
			cout << indent_str_next << "Area = " << area.get_area()*1e-6  << " mm^2" << endl;
			cout << indent_str_next << "per_access_energy = " << per_access_energy * 1e+12 << " pJ" << endl;
//			cout << indent_str_next << "base_energy = " << base_energy * 1e+9 << " nJ" << endl;
			cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage  << " W" << endl;
			cout << indent_str_next<< "Subthreshold Leakage = "
						<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
							<< (power.readOp.power_gated_leakage * (long_channel? power.readOp.longer_channel_leakage/power.readOp.leakage:1) )  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
		}
		else if (fu_type == MUL)
		{
			cout << indent_str << "Complex ALUs (Mul/Div) (Count: "<< coredynp.num_muls <<" ):" << endl;
			cout << indent_str_next << "Area = " << area.get_area()*1e-6  << " mm^2" << endl;
			cout << indent_str_next << "per_access_energy = " << per_access_energy * 1e+12 << " pJ" << endl;
//			cout << indent_str_next << "base_energy = " << base_energy * 1e+9 << " nJ" << endl;
			cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate  << " W" << endl;
//			cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage  << " W" << endl;
			cout << indent_str_next<< "Subthreshold Leakage = "
						<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
			if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
							<< (power.readOp.power_gated_leakage * (long_channel? power.readOp.longer_channel_leakage/power.readOp.leakage:1) )  << " W" << endl;
			cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage  << " W" << endl;
			cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
			cout <<endl;
		}
	}
	else
	{
	}
}

void FunctionalUnit::leakage_feedback(double temperature)
{
  // Update the temperature and initialize the global interfaces.
  interface_ip.temp = (unsigned int)round(temperature/10.0)*10;
  if(debug)
  cout << "FunctionalU leakage_feedback(), techsize: " << interface_ip.F_sz_um << ", wire: " << interface_ip.wire_F_sz_um << ", finfet: " << interface_ip.is_finfet << ", ncfet: " << interface_ip.is_ncfet <<
		  ", itrs: " << interface_ip.is_itrs2012 << ", asap7: " << interface_ip.is_asap7 << ", projection: " << interface_ip.ic_proj_type << ", vdd: " << interface_ip.vdd << endl;

  uca_org_t init_result = init_interface(&interface_ip); // init_result is dummy

  // This is part of FunctionalUnit()
  double area_t, leakage, gate_leakage;
  double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

  if (fu_type == FPU)
  {
	area_t = 4.47*1e6*(g_ip->F_sz_nm*g_ip->F_sz_nm/90.0/90.0);//this is um^2 The base number
	if (g_ip->F_sz_nm>90)
		area_t = 4.47*1e6*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2
	leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
	gate_leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
  }
  else if (fu_type == ALU)
  {
    area_t = 280*260*2*num_fu*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
  }
  else if (fu_type == MUL)
  {
    area_t = 280*260*2*3*num_fu*g_tp.scaling_factor.logic_scaling_co_eff;//this is um^2 ALU + MUl
    leakage = area_t *(g_tp.scaling_factor.core_tx_density)*cmos_Isub_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;//unit W
    gate_leakage = area_t*(g_tp.scaling_factor.core_tx_density)*cmos_Ig_leakage(20*g_tp.min_w_nmos_, 20*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd/2;
  }
  else
  {
    cout<<"Unknown Functional Unit Type"<<endl;
    exit(1);
  }

  power.readOp.leakage = leakage*num_fu;
  power.readOp.gate_leakage = gate_leakage*num_fu;
  power.readOp.longer_channel_leakage = longer_channel_device_reduction(Core_device, coredynp.core_ty)*power.readOp.leakage;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;

}

UndiffCore::UndiffCore(ParseXML* XML_interface, int ithCore_, InputParameter* interface_ip_, const CoreDynParam & dyn_p_, bool exist_,  bool embedded_)
:XML(XML_interface),
 ithCore(ithCore_),
 interface_ip(*interface_ip_),
 coredynp(dyn_p_),
 core_ty(coredynp.core_ty),
 embedded(XML->sys.Embedded),
 pipeline_stage(coredynp.pipeline_stages),
 num_hthreads(coredynp.num_hthreads),
 issue_width(coredynp.issueW),
 exist(exist_)
// is_default(_is_default)
{
	if (!exist) return;

	double undifferentiated_core=0;
	double core_tx_density=0;
	double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
	double undifferentiated_core_coe;
	//XML_interface=_XML_interface;
	uca_org_t result2;
	  if(debug)
	  cout << "UndiffCore, techsize: " << interface_ip.F_sz_um << ", wire: " << interface_ip.wire_F_sz_um << ", finfet: " << interface_ip.is_finfet << ", ncfet: " << interface_ip.is_ncfet <<
			  ", itrs: " << interface_ip.is_itrs2012 << ", asap7: " << interface_ip.is_asap7 << ", projection: " << interface_ip.ic_proj_type << ", vdd: " << interface_ip.vdd << endl;

	result2 = init_interface(&interface_ip);

	//Compute undifferentiated core area at 90nm.
	if (embedded==false)
	{
		//Based on the results of polynomial/log curve fitting based on undifferentiated core of Niagara, Niagara2, Merom, Penyrn, Prescott, Opteron die measurements
		if (core_ty==OOO)
		{
			//undifferentiated_core = (0.0764*pipeline_stage*pipeline_stage -2.3685*pipeline_stage + 10.405);//OOO
			undifferentiated_core = (3.57*log(pipeline_stage)-1.2643)>0?(3.57*log(pipeline_stage)-1.2643):0;
		}
		else if (core_ty==Inorder)
		{
			//undifferentiated_core = (0.1238*pipeline_stage + 7.2572)*0.9;//inorder
			undifferentiated_core = (-2.19*log(pipeline_stage)+6.55)>0?(-2.19*log(pipeline_stage)+6.55):0;
		}
		else
		{
			cout<<"invalid core type"<<endl;
			exit(0);
		}
		undifferentiated_core *= (1+ logtwo(num_hthreads)* 0.0716);
	}
	else
	{
		//Based on the results in paper "parametrized processor models" Sandia Labs
		if (XML->sys.opt_clockrate)
			undifferentiated_core_coe = 0.05;
		else
			undifferentiated_core_coe = 0;
		undifferentiated_core = (0.4109* pipeline_stage - 0.776)*undifferentiated_core_coe;
		undifferentiated_core *= (1+ logtwo(num_hthreads)* 0.0426);
	}
	if(debug)
		cout << ", pipeline_stage: " << pipeline_stage <<
			", undifferentiated_core: " << undifferentiated_core <<
			", g_tp.scaling_factor.logic_scaling_co_eff: " << g_tp.scaling_factor.logic_scaling_co_eff <<
			", core_tx_density: " << core_tx_density << endl;
	undifferentiated_core 		    *= g_tp.scaling_factor.logic_scaling_co_eff*1e6;//change from mm^2 to um^2
	core_tx_density                 = g_tp.scaling_factor.core_tx_density;
	if(debug)
		cout << "core_tx_density: " << core_tx_density << endl;
//divya removing undiff core 10-4-2022
/*
	power.readOp.leakage = undifferentiated_core*(core_tx_density)*cmos_Isub_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd;//unit W
	power.readOp.gate_leakage = undifferentiated_core*(core_tx_density)*cmos_Ig_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd;

	double long_channel_device_reduction = longer_channel_device_reduction(Core_device, coredynp.core_ty);
	power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;

	double pg_reduction = power_gating_leakage_reduction(false);
	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;


	area.set_area(undifferentiated_core);

	scktRatio = g_tp.sckt_co_eff;
	power.readOp.dynamic *= scktRatio;
	power.writeOp.dynamic *= scktRatio;
	power.searchOp.dynamic *= scktRatio;
	macro_PR_overhead = g_tp.macro_layout_overhead;
	area.set_area(area.get_area()*macro_PR_overhead);
*/
	undifferentiated_core = 0;
	power.readOp.leakage = 0; //undifferentiated_core*(core_tx_density)*cmos_Isub_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd;//unit W
	power.readOp.gate_leakage = 0; //undifferentiated_core*(core_tx_density)*cmos_Ig_leakage(5*g_tp.min_w_nmos_, 5*g_tp.min_w_nmos_*pmos_to_nmos_sizing_r, 1, inv)*g_tp.peri_global.Vdd;

	double long_channel_device_reduction = 0; //longer_channel_device_reduction(Core_device, coredynp.core_ty);
	power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;

	double pg_reduction = 0; //power_gating_leakage_reduction(false);
	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;

	area.set_area(undifferentiated_core);

	scktRatio = g_tp.sckt_co_eff;
	power.readOp.dynamic = 0; //*= scktRatio;
	power.writeOp.dynamic = 0; //*= scktRatio;
	power.searchOp.dynamic = 0; //*= scktRatio;
	macro_PR_overhead = g_tp.macro_layout_overhead;
	area.set_area(area.get_area()*macro_PR_overhead);
}


void UndiffCore::displayEnergy(uint32_t indent,int plevel,bool is_tdp)
{
	string indent_str(indent, ' ');
	string indent_str_next(indent+2, ' ');
	bool long_channel = XML->sys.longer_channel_device;
	bool power_gating = XML->sys.power_gating;

	if (is_tdp)
	{
		cout << indent_str << "UndiffCore:" << endl;
		cout << indent_str_next << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate << " W" << endl;
		//cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage <<" W" << endl;
		cout << indent_str_next<< "Subthreshold Leakage = "
					<< (long_channel? power.readOp.longer_channel_leakage:power.readOp.leakage) <<" W" << endl;
		if (power_gating) cout << indent_str_next << "Subthreshold Leakage with power gating = "
						<< (power.readOp.power_gated_leakage * (long_channel? power.readOp.longer_channel_leakage/power.readOp.leakage:1) )  << " W" << endl;
		cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
		//cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}
	else
	{
		cout << indent_str << "UndiffCore:" << endl;
		cout << indent_str_next << "Area = " << area.get_area()*1e-6<< " mm^2" << endl;
		cout << indent_str_next << "Peak Dynamic = " << power.readOp.dynamic*clockRate << " W" << endl;
		cout << indent_str_next << "Subthreshold Leakage = " << power.readOp.leakage <<" W" << endl;
		cout << indent_str_next << "Gate Leakage = " << power.readOp.gate_leakage << " W" << endl;
		//cout << indent_str_next << "Runtime Dynamic = " << rt_power.readOp.dynamic/executionTime << " W" << endl;
		cout <<endl;
	}

}

inst_decoder::inst_decoder(
		bool   _is_default,
		const InputParameter *configure_interface,
		int opcode_length_,
		int num_decoders_,
		bool x86_,
	    enum Device_ty device_ty_,
	    enum Core_type core_ty_)
:is_default(_is_default),
 opcode_length(opcode_length_),
 num_decoders(num_decoders_),
 x86(x86_),
 device_ty(device_ty_),
 core_ty(core_ty_)
 {
			/*
			 * Instruction decoder is different from n to 2^n decoders
			 * that are commonly used in row decoders in memory arrays.
			 * The RISC instruction decoder is typically a very simple device.
			 * We can decode an instruction by simply
			 * separating the machine word into small parts using wire slices
			 * The RISC instruction decoder can be approximate by the n to 2^n decoders,
			 * although this approximation usually underestimate power since each decoded
			 * instruction normally has more than 1 active signal.
			 *
			 * However, decoding a CISC instruction word is much more difficult
			 * than the RISC case. A CISC decoder is typically set up as a state machine.
			 * The machine reads the opcode field to determine
			 * what type of instruction it is,
			 * and where the other data values are.
			 * The instruction word is read in piece by piece,
			 * and decisions are made at each stage as to
			 * how the remainder of the instruction word will be read.
			 * (sequencer and ROM are usually needed)
			 * An x86 decoder can be even more complex since
			 * it involve  both decoding instructions into u-ops and
			 * merge u-ops when doing micro-ops fusion.
			 */
			bool is_dram=false;
			double pmos_to_nmos_sizing_r;
			double load_nmos_width, load_pmos_width;
			double C_driver_load, R_wire_load;
			Area cell;

			l_ip=*configure_interface;
			  if(debug)
			  cout << "Decoder, techsize: " << l_ip.F_sz_um << ", wire: " << l_ip.wire_F_sz_um << ", finfet: " << l_ip.is_finfet << ", ncfet: " << l_ip.is_ncfet <<
					  ", itrs: " << l_ip.is_itrs2012 << ", asap7: " << l_ip.is_asap7 << ", projection: " << l_ip.ic_proj_type << ", vdd: " << l_ip.vdd << endl;

			local_result = init_interface(&l_ip);
			cell.h =g_tp.cell_h_def;
			cell.w =g_tp.cell_h_def;

			num_decoder_segments = (int)ceil(opcode_length/18.0);
			if (opcode_length > 18)	opcode_length = 18;
			num_decoded_signals= (int)pow(2.0,opcode_length);
			pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
			load_nmos_width=g_tp.max_w_nmos_ /2;
			load_pmos_width= g_tp.max_w_nmos_ * pmos_to_nmos_sizing_r;
			C_driver_load = 1024*gate_C(load_nmos_width + load_pmos_width, 0, is_dram); //TODO: this number 1024 needs to be revisited
			R_wire_load   = 3000*l_ip.F_sz_um * g_tp.wire_outside_mat.R_per_um;

			final_dec = new Decoder(
					num_decoded_signals,
					false,
					C_driver_load,
					R_wire_load,
					false/*is_fa*/,
					false/*is_dram*/,
					false/*wl_tr*/, //to use peri device
					cell);

			PredecBlk * predec_blk1 = new PredecBlk(
					num_decoded_signals,
					final_dec,
					0,//Assuming predec and dec are back to back
					0,
					1,//Each Predec only drives one final dec
					false/*is_dram*/,
					true);
			PredecBlk * predec_blk2 = new PredecBlk(
					num_decoded_signals,
					final_dec,
					0,//Assuming predec and dec are back to back
					0,
					1,//Each Predec only drives one final dec
					false/*is_dram*/,
					false);

			PredecBlkDrv * predec_blk_drv1 = new PredecBlkDrv(0, predec_blk1, false);
			PredecBlkDrv * predec_blk_drv2 = new PredecBlkDrv(0, predec_blk2, false);

			pre_dec            = new Predec(predec_blk_drv1, predec_blk_drv2);

			double area_decoder = final_dec->area.get_area() * num_decoded_signals * num_decoder_segments*num_decoders;
			//double w_decoder    = area_decoder / area.get_h();
			double area_pre_dec = (predec_blk_drv1->area.get_area() +
					predec_blk_drv2->area.get_area() +
					predec_blk1->area.get_area() +
					predec_blk2->area.get_area())*
					num_decoder_segments*num_decoders;
			area.set_area(area.get_area()+ area_decoder + area_pre_dec);
			double macro_layout_overhead   = g_tp.macro_layout_overhead;
			double chip_PR_overhead        = g_tp.chip_layout_overhead;
			area.set_area(area.get_area()*macro_layout_overhead*chip_PR_overhead);

			inst_decoder_delay_power();

			double sckRation = g_tp.sckt_co_eff;
			power.readOp.dynamic *= sckRation;
			power.writeOp.dynamic *= sckRation;
			power.searchOp.dynamic *= sckRation;

			double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);
			power.readOp.longer_channel_leakage	= power.readOp.leakage*long_channel_device_reduction;

	    	double pg_reduction = power_gating_leakage_reduction(false);
	    	power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;


}

void inst_decoder::inst_decoder_delay_power()
{

	double dec_outrisetime;
	double inrisetime=0, outrisetime;
	double pppm_t[4]    = {1,1,1,1};
	double squencer_passes = x86?2:1;

	outrisetime = pre_dec->compute_delays(inrisetime);
	dec_outrisetime = final_dec->compute_delays(outrisetime);
	set_pppm(pppm_t, squencer_passes*num_decoder_segments, num_decoder_segments, squencer_passes*num_decoder_segments, num_decoder_segments);
    power = power + pre_dec->power*pppm_t;
    set_pppm(pppm_t, squencer_passes*num_decoder_segments, num_decoder_segments*num_decoded_signals,
    		num_decoder_segments*num_decoded_signals, squencer_passes*num_decoder_segments);
    power = power + final_dec->power*pppm_t;
}

void inst_decoder::leakage_feedback(double temperature)
{
  l_ip.temp = (unsigned int)round(temperature/10.0)*10;
  cout << "inst_decoder leakage_feedback(), techsize: " << l_ip.F_sz_um << ", wire: " << l_ip.wire_F_sz_um << ", finfet: " << l_ip.is_finfet << ", ncfet: " << l_ip.is_ncfet <<
		  ", itrs: " << l_ip.is_itrs2012 << ", asap7: " << l_ip.is_asap7 << ", projection: " << l_ip.ic_proj_type << ", vdd: " << l_ip.vdd << endl;
  uca_org_t init_result = init_interface(&l_ip); // init_result is dummy

  final_dec->leakage_feedback(temperature);
  pre_dec->leakage_feedback(temperature);

  double pppm_t[4]    = {1,1,1,1};
  double squencer_passes = x86?2:1;

  set_pppm(pppm_t, squencer_passes*num_decoder_segments, num_decoder_segments, squencer_passes*num_decoder_segments, num_decoder_segments);
  power = pre_dec->power*pppm_t;

  set_pppm(pppm_t, squencer_passes*num_decoder_segments, num_decoder_segments*num_decoded_signals,num_decoder_segments*num_decoded_signals, squencer_passes*num_decoder_segments);
  power = power + final_dec->power*pppm_t;

  double sckRation = g_tp.sckt_co_eff;

  power.readOp.dynamic *= sckRation;
  power.writeOp.dynamic *= sckRation;
  power.searchOp.dynamic *= sckRation;

  double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);
  power.readOp.longer_channel_leakage = power.readOp.leakage*long_channel_device_reduction;

  double pg_reduction = power_gating_leakage_reduction(false);
  power.readOp.power_gated_leakage	= power.readOp.leakage*pg_reduction;

}

inst_decoder::~inst_decoder()
{
	  local_result.cleanup();

	  delete final_dec;

	  delete pre_dec->blk1;
	  delete pre_dec->blk2;
	  delete pre_dec->drv1;
	  delete pre_dec->drv2;
	  delete pre_dec;
}
