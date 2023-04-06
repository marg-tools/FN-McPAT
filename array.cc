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

#define  GLOBALVAR
#include "area.h"
#include "decoder.h"
#include "parameter.h"
#include "array.h"
#include <iostream>
#include <math.h>
#include <assert.h>
#include "globalvar.h"
//#include "scaling_factors_45nm_to_14nm.h"
#define debug_ 0
using namespace std;

ArrayST::ArrayST(const InputParameter *configure_interface,
		               string _name,
		               enum Device_ty device_ty_,
		               bool opt_local_,
		               enum Core_type core_ty_,
		               bool _is_default)
:l_ip(*configure_interface),
 name(_name),
 device_ty(device_ty_),
 opt_local(opt_local_),
 core_ty(core_ty_),
 is_default(_is_default)
{
	if (l_ip.cache_sz<64) l_ip.cache_sz=64;
	if (l_ip.power_gating && (l_ip.assoc==0)) { l_ip.power_gating = false;}
	l_ip.error_checking();//not only do the error checking but also fill some missing parameters

	optimize_array();

}


void ArrayST::compute_base_power()
{
/*	  l_ip.ed			= 1;		//0-E, 1-ED, 2-ED^2

	if(name == "L2cache") //If L2 is LLC
	{
//		  cout << "size: " << l_ip.cache_sz << ", banks: " << l_ip.nbanks << ", vdd: " << l_ip.vdd << endl;
		  l_ip.add_ecc_b_ = true;
		  l_ip.tag_w = 37;
		  l_ip.wt = Low_swing;	//5;
		  l_ip.wire_is_mat_type = 1; //Semi-global
		  l_ip.wire_os_mat_type = 2; //Global
		  l_ip.force_wiretype = 1;
		  l_ip.force_cache_config = 1;

		  if(l_ip.cache_sz == 2097152) {
			  l_ip.ndwl = 2;
			  l_ip.ndbl = 8;	//2;
			  l_ip.nspd = 2;	//4;
			  l_ip.ndcm = 8;
			  l_ip.ndsam1 = 1;
			  l_ip.ndsam2 = 1;

			  l_ip.ntwl = 2;
			  l_ip.ntbl = 2;
			  l_ip.ntspd = 1;
			  l_ip.ntcm = 1;
			  l_ip.ntsam1 = 2;
			  l_ip.ntsam2 = 1;
		  }
		  else
			  l_ip.force_cache_config = 0;
	}
*/
	local_result=cacti_interface(&l_ip);

	  if(debug_)
//	  if(name == "L2cache" || name == "L2MissB" || name == "L2FillB" || name == "L2PrefetchB" || name == "L2WBB")
//	  if(name == "Integer_Register_File" || name == "Floating_point_Register_File")
	  {
		  cout << "array.cc:: name: " << name << ", size: " << l_ip.cache_sz << ", assoc: " << l_ip.assoc << ", banks: " << l_ip.nbanks << ", levels: " << l_ip.cache_level << endl;
		  l_ip.display_ip();
		  output_UCA(&local_result);
	  }

}

void ArrayST::optimize_array()
{
	list<uca_org_t > candidate_solutions(0);
	list<uca_org_t >::iterator candidate_iter, min_dynamic_energy_iter;

	uca_org_t * temp_res = 0;
	local_result.valid=false;

	double 	throughput=l_ip.throughput, latency=l_ip.latency;
	double  area_efficiency_threshold = 20.0;
	bool 	throughput_overflow=true, latency_overflow=true;
	int     optimization_end = 20;
	compute_base_power();

	if(debug_)
	if(name == "L2cache") {
		cout << "L2_cycletime: " << local_result.cycle_time << ", throughput: " << throughput << ", throughput_overflow: " << throughput_overflow << " : " << local_result.cycle_time - throughput <<
				"\n" << ", L3_accesstime: " << local_result.access_time << ", latency: " << latency << ", latency_overflow: " <<latency_overflow << " : " << local_result.access_time - latency << endl;
	}

	if ((local_result.cycle_time - throughput) <= 1e-10 )
		throughput_overflow=false;
	if ((local_result.access_time - latency)<= 1e-10)
		latency_overflow=false;

	if ((opt_for_clk && opt_local) && ((l_ip.cache_sz>2048 && l_ip.assoc!=0) ||(l_ip.cache_sz>256 && l_ip.assoc==0)))//over opt small array lead to sub-optimal solutions
	{
		if (throughput_overflow || latency_overflow)
		{
			l_ip.ed=0;

			l_ip.delay_wt                = 100;//Fixed number, make sure timing can be satisfied.
			l_ip.cycle_time_wt           = 1000;

			l_ip.area_wt                 = 10;//Fixed number, This is used to exhaustive search for individual components.
			l_ip.dynamic_power_wt        = 10;//Fixed number, This is used to exhaustive search for individual components.
			l_ip.leakage_power_wt        = 10;

			l_ip.delay_dev               = 1000000;//Fixed number, make sure timing can be satisfied.
			l_ip.cycle_time_dev          = 100;

			l_ip.area_dev                = 1000000;//Fixed number, This is used to exhaustive search for individual components.
			l_ip.dynamic_power_dev       = 1000000;//Fixed number, This is used to exhaustive search for individual components.
			l_ip.leakage_power_dev       = 1000000;

			throughput_overflow=true; //Reset overflow flag before start optimization iterations
			latency_overflow=true;

			temp_res = &local_result; //Clean up the result for optimized for ED^2P
			temp_res->cleanup();
		}


		while ((throughput_overflow || latency_overflow)&&l_ip.cycle_time_dev > optimization_end)// l_ip.delay_dev <40 will have over-opt results
		{
			compute_base_power();

			l_ip.cycle_time_dev-=10;//This is the time_dev to be used for next iteration

			//		from best area to worst area -->worst timing to best timing
			if ((((local_result.cycle_time - throughput) <= 1e-10 ) && (local_result.access_time - latency)<= 1e-10)||
					(local_result.data_array2.area_efficiency < area_efficiency_threshold && l_ip.assoc == 0))
			{  //if no satisfiable solution is found,the most aggressive one is left
				candidate_solutions.push_back(local_result);
				//output_data_csv(candidate_solutions.back());
				if (((local_result.cycle_time - throughput) <= 1e-10) && ((local_result.access_time - latency)<= 1e-10))
					//ensure stop opt not because of cam
				{
					throughput_overflow=false;
					latency_overflow=false;
				}

			}
			else
			{
				//TODO: whether checking the partial satisfied results too, or just change the mark???
				if ((local_result.cycle_time - throughput) <= 1e-10)
										throughput_overflow=false;
				if ((local_result.access_time - latency)<= 1e-10)
										latency_overflow=false;

				if (l_ip.cycle_time_dev > optimization_end)
				{   //if not >10 local_result is the last result, it cannot be cleaned up
					temp_res = &local_result; //Only solutions not saved in the list need to be cleaned up
					temp_res->cleanup();
				}
			}
		}

		if (l_ip.assoc > 0)
		{
			//For array structures except CAM and FA, Give warning but still provide a result with best timing found
			if (throughput_overflow==true)
				cout<< "Warning: " << name<<" array structure cannot satisfy throughput constraint." << endl;
			if (latency_overflow==true)
				cout<< "Warning: " << name<<" array structure cannot satisfy latency constraint." << endl;
		}

		//double min_dynamic_energy, min_dynamic_power, min_leakage_power, min_cycle_time;
		double min_dynamic_energy=BIGNUM;
		if (candidate_solutions.empty()==false)
		{
			local_result.valid=true;
			for (candidate_iter = candidate_solutions.begin(); candidate_iter != candidate_solutions.end(); ++candidate_iter)
			{
				if (min_dynamic_energy > (candidate_iter)->power.readOp.dynamic)
				{
					min_dynamic_energy = (candidate_iter)->power.readOp.dynamic;
					min_dynamic_energy_iter = candidate_iter;
					local_result = *(min_dynamic_energy_iter);
					//TODO: since results are reordered results and l_ip may miss match. Therefore, the final output spread sheets may show the miss match.
				}
				else
				{
					candidate_iter->cleanup() ;
				}
			}
		}
	candidate_solutions.clear();
	}

	if(debug_)
	if(name == "L2cache")
		cout << "array.cc:: name: " << name << ", size: " << l_ip.cache_sz
				  << ", leakage: " << local_result.power.readOp.leakage << ", dynamic: " << local_result.power.readOp.dynamic
				  << ", access_time: " << local_result.access_time << ", latency: " << latency << endl;

	double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);
	double pg_reduction = power_gating_leakage_reduction(true);//array structure all retain state;

	double macro_layout_overhead   = g_tp.macro_layout_overhead;
	double chip_PR_overhead        = g_tp.chip_layout_overhead;
	double total_overhead          = macro_layout_overhead*chip_PR_overhead;
	local_result.area *= total_overhead;

	//maintain constant power density
	double pppm_t[4]    = {total_overhead,1,1,total_overhead};

	double sckRation = g_tp.sckt_co_eff;
	local_result.power.readOp.dynamic *= sckRation;
	local_result.power.writeOp.dynamic *= sckRation;
	local_result.power.searchOp.dynamic *= sckRation;
	local_result.power.readOp.leakage *= l_ip.nbanks;
	local_result.power.readOp.longer_channel_leakage =
		local_result.power.readOp.leakage*long_channel_device_reduction;

	if (l_ip.assoc==0)//only use this function for CAM/FA since other array types compute pg leakage automatically
	{
		local_result.power.readOp.power_gated_leakage =
			local_result.power.readOp.leakage*pg_reduction;
	}
	else
	{
		local_result.power.readOp.power_gated_leakage *= l_ip.nbanks;//normal array types
	}

	local_result.power = local_result.power* pppm_t;


	local_result.data_array2.power.readOp.dynamic *= sckRation;
	local_result.data_array2.power.writeOp.dynamic *= sckRation;
	local_result.data_array2.power.searchOp.dynamic *= sckRation;
	local_result.data_array2.power.readOp.leakage *= l_ip.nbanks;
	local_result.data_array2.power.readOp.longer_channel_leakage =
		local_result.data_array2.power.readOp.leakage*long_channel_device_reduction;
	if (l_ip.assoc==0)//only use this function for CAM/FA since other array types compute pg leakage automatically
	{
		local_result.data_array2.power.readOp.power_gated_leakage =
			local_result.data_array2.power.readOp.leakage*pg_reduction;
	}
	else
	{
		local_result.data_array2.power.readOp.power_gated_leakage *= l_ip.nbanks;//normal array types
	}
	local_result.data_array2.power = local_result.data_array2.power* pppm_t;

	if (!(l_ip.pure_cam || l_ip.pure_ram || l_ip.fully_assoc) && l_ip.is_cache)
	{
		local_result.tag_array2.power.readOp.dynamic *= sckRation;
		local_result.tag_array2.power.writeOp.dynamic *= sckRation;
		local_result.tag_array2.power.searchOp.dynamic *= sckRation;
		local_result.tag_array2.power.readOp.leakage *= l_ip.nbanks;
		local_result.data_array2.power.readOp.power_gated_leakage *= l_ip.nbanks;
		local_result.tag_array2.power.readOp.longer_channel_leakage =
			local_result.tag_array2.power.readOp.leakage*long_channel_device_reduction;
		local_result.tag_array2.power = local_result.tag_array2.power* pppm_t;
	}
//	cout << "unicache area: " << local_result.area << endl;

	if(debug_)
	if(name == "L2cache") {
		  cout << "array.cc: sckt_coeff : " << sckRation << endl;

		cout << "array.cc:: name: " << name << ", size: " << l_ip.cache_sz
				  << ", leakage: " << local_result.power.readOp.leakage << ", dynamic: " << local_result.power.readOp.dynamic
				  << ", access_time: " << local_result.access_time << ", latency: " << latency << endl;
	}
//Since RegFiles are made of HighPerforance cells, below scaling factors are used.
//Scaling factors are obtained from CACTI results at 22-nm and from https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7223683
	if(name == "Integer_Register_File" || name == "Floating_point_Register_File" )
	{
		local_result.area *= 1.42; //2;
		local_result.power.readOp.dynamic *= 1.35; //4;
		local_result.power.writeOp.dynamic *= 1.58; //4;
		local_result.power.searchOp.dynamic *= 1.7; //4;
		local_result.power.readOp.leakage *= 12.76;
		local_result.data_array2.power.readOp.dynamic *= 1.35;
		local_result.data_array2.power.writeOp.dynamic *= 1.58;
		local_result.data_array2.power.searchOp.dynamic *= 1.7;
		local_result.data_array2.power.readOp.leakage *= 12.76;
		local_result.tag_array2.power.readOp.dynamic *= 1.35;
		local_result.tag_array2.power.writeOp.dynamic *= 1.58;
		local_result.tag_array2.power.searchOp.dynamic *= 1.7;
		local_result.tag_array2.power.readOp.leakage *= 12.76;
		local_result.data_array2.width *= 1.19;
		local_result.data_array2.height *= 1.19;
		local_result.tag_array2.width *= 1.19;
		local_result.tag_array2.height *= 1.19;
		local_result.cache_ht *= 1.19;
		local_result.cache_len *= 1.19;
	}
}

void ArrayST::leakage_feedback(double temperature)//TODO: add the code to process power-gating leakage
{
  // Update the temperature. l_ip is already set and error-checked in the creator function.
  l_ip.temp = (unsigned int)round(temperature/10.0)*10;

  // This corresponds to cacti_interface() in the initialization process. Leakage power is updated here.
  reconfigure(&l_ip,&local_result);

  // Scale the power values. This is part of ArrayST::optimize_array().
  double long_channel_device_reduction = longer_channel_device_reduction(device_ty,core_ty);

  double macro_layout_overhead   = g_tp.macro_layout_overhead;
  double chip_PR_overhead        = g_tp.chip_layout_overhead;
  double total_overhead          = macro_layout_overhead*chip_PR_overhead;

  double pppm_t[4]    = {total_overhead,1,1,total_overhead};

  double sckRation = g_tp.sckt_co_eff;
  cout << "array.cc: sckt_coeff : " << sckRation << endl;

  local_result.power.readOp.dynamic *= sckRation;
  local_result.power.writeOp.dynamic *= sckRation;
  local_result.power.searchOp.dynamic *= sckRation;
  local_result.power.readOp.leakage *= l_ip.nbanks;
  local_result.power.readOp.longer_channel_leakage = local_result.power.readOp.leakage*long_channel_device_reduction;
  local_result.power = local_result.power* pppm_t;

  local_result.data_array2.power.readOp.dynamic *= sckRation;
  local_result.data_array2.power.writeOp.dynamic *= sckRation;
  local_result.data_array2.power.searchOp.dynamic *= sckRation;
  local_result.data_array2.power.readOp.leakage *= l_ip.nbanks;
  local_result.data_array2.power.readOp.longer_channel_leakage = local_result.data_array2.power.readOp.leakage*long_channel_device_reduction;
  local_result.data_array2.power = local_result.data_array2.power* pppm_t;

  if (!(l_ip.pure_cam || l_ip.pure_ram || l_ip.fully_assoc) && l_ip.is_cache)
  {
    local_result.tag_array2.power.readOp.dynamic *= sckRation;
    local_result.tag_array2.power.writeOp.dynamic *= sckRation;
    local_result.tag_array2.power.searchOp.dynamic *= sckRation;
    local_result.tag_array2.power.readOp.leakage *= l_ip.nbanks;
    local_result.tag_array2.power.readOp.longer_channel_leakage = local_result.tag_array2.power.readOp.leakage*long_channel_device_reduction;
    local_result.tag_array2.power = local_result.tag_array2.power* pppm_t;
  }
}

ArrayST:: ~ArrayST()
{
	local_result.cleanup();
}
