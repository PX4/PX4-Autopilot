/*
	Application name:       FUzzy Development and Generation Environment (FUDGE) Version V1.00
	File name:                      Fuzzy.h
	Written by:                     Alex DeCastro & Jason Spielman

	Copyright Motorola 1994
*/
#ifndef FuzzyFB_h
#define FuzzyFB_h

#define TRACE          0      /*set to display fuzzy parameters */
#define NO_RULES       0      /*set to display inputs when no rules fire*/
#define MIN(A,B)       (A < B) ? A : B
#define MAX(A,B)       (A > B) ? A : B

struct In {
	float           min;
	float           max;
};
struct Out {
	float           min;
	float           max;
};
struct Rule {
	int             antecedent[8];
	int             consequent[8];
};

class FuzzyFB
{
    public:
     FuzzyFB();
     void fuzzy_step(int *crisp_inuts, int *crisp_oututs);
     
     int num_inputs;
     int num_outputs;
     int num_rules;
     Out *Outputs;
     int *num_output_mfs;
     int *num_input_mfs;
     In *Inputs;
     Rule *Rules;
     int *num_rule_ants ;
     int *num_rule_cons; 
     float (*inmem_points)[7][4];
     float (*outmem_points)[7][4];

    private:
      void    fuzzify_input(int in_index,int in_val);
      float   get_membership_value(int in_index,int mf_index,int in_val);
      void    eval_rule(int rule_index);
      int     defuzzify_output(int out_index);
      float   fuzzy_inputs[8][8];
      float   fuzzy_outputs[4][8];
      float   rule_strengths[64];

};
#endif
