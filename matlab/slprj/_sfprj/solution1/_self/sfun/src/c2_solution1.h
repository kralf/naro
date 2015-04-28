#ifndef __c2_solution1_h__
#define __c2_solution1_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_solution1InstanceStruct
#define typedef_SFc2_solution1InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_solution1;
  uint32_T c2_method;
  boolean_T c2_method_not_empty;
  uint32_T c2_state;
  boolean_T c2_state_not_empty;
  uint32_T c2_b_state[2];
  boolean_T c2_b_state_not_empty;
  uint32_T c2_c_state[625];
  boolean_T c2_c_state_not_empty;
  real_T (*c2_u)[3];
  real_T *c2_y;
} SFc2_solution1InstanceStruct;

#endif                                 /*typedef_SFc2_solution1InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_solution1_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_solution1_get_check_sum(mxArray *plhs[]);
extern void c2_solution1_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
