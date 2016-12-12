#include <math.h>
#include "FuzzyFB.h"


FuzzyFB::FuzzyFB()
{
  
}

void FuzzyFB::fuzzy_step(int *crisp_inputs, int *crisp_outputs)
{
	int     in_index,rule_index,out_index;

	for (in_index = 0;in_index < num_inputs;in_index++)
	{
		fuzzify_input(in_index,crisp_inputs[in_index]);
	}
	for (rule_index = 0;rule_index < num_rules;rule_index++)
	{
		eval_rule(rule_index);
	}
	for (out_index = 0;out_index < num_outputs;out_index++)
	{
		crisp_outputs[out_index] = defuzzify_output(out_index);
	}
}
void FuzzyFB::fuzzify_input(int in_index,int in_val)
{
	int i;
	for (i = 0;i < num_input_mfs[in_index];i++)
		fuzzy_inputs[in_index][i] = get_membership_value(in_index,i,in_val);

}
float FuzzyFB::get_membership_value(int in_index,int mf_index,int in_val)
{
	if (in_val < inmem_points[in_index][mf_index][0]) return 0;
	if (in_val > inmem_points[in_index][mf_index][3]) return 0;
	if (in_val <= inmem_points[in_index][mf_index][1])
	{
		if (fabs(inmem_points[in_index][mf_index][0] - inmem_points[in_index][mf_index][1])<0.1)
			return 1;
		else
			return ((in_val - inmem_points[in_index][mf_index][0]) /
					(inmem_points[in_index][mf_index][1] - inmem_points[in_index][mf_index][0]));
	}
	if (in_val >= inmem_points[in_index][mf_index][2])
	{
		if (fabs(inmem_points[in_index][mf_index][2] - inmem_points[in_index][mf_index][3])<0.1)
			return 1;
		else
			return ((inmem_points[in_index][mf_index][3] - in_val) /
					(inmem_points[in_index][mf_index][3] - inmem_points[in_index][mf_index][2]));
	}
	return 1;
}
void FuzzyFB::eval_rule(int rule_index)
{
	int             in_index,out_index,mf_index,ant_index,con_index;
	int     val;
	float   rule_strength = 1;
	for     (ant_index = 0;ant_index < num_rule_ants[rule_index];ant_index++)
	{
		val = Rules[rule_index].antecedent[ant_index];
		in_index = (val & 0x07);
		mf_index = ((val & 0x38) >> 3);
		rule_strength = MIN(rule_strength,fuzzy_inputs[in_index][mf_index]);
	}
	rule_strengths[rule_index] = rule_strength;
	for (con_index = 0;con_index < num_rule_cons[rule_index];con_index++)
	{
		val = Rules[rule_index].consequent[con_index];
		out_index = (val & 0x03);
		mf_index = ((val & 0x38) >> 3);
		fuzzy_outputs[out_index][mf_index] = MAX(fuzzy_outputs[out_index][mf_index],
			rule_strengths[rule_index]);
	}
}

int FuzzyFB::defuzzify_output(int out_index)
{
	float           summ = 0;
	float           product = 0;
	float           temp1,temp2;
	int             mf_index;
	for (mf_index = 0;mf_index < num_output_mfs[out_index];mf_index++)
	{
		temp1 = fuzzy_outputs[out_index][mf_index];
		temp2 = outmem_points[out_index][mf_index][0];
		summ = summ + temp1;
		product = product + (temp1 * temp2);
		fuzzy_outputs[out_index][mf_index] = 0;
	}
	if (summ > 0)
		return(int)(product / summ);
	else
	        return 0;
	 
}
