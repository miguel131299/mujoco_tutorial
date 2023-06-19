#include <stdio.h>
#include <math.h>

#include "nlopt.h"

extern mjModel* m;
extern mjData* dsim;

double x_target, z_target;

/*Optimization problem

Cost = 0;

equality constraints:
  ceq_1 = x_target - sensordata[0];
  ceq_2 = z_target - sensordata[2];

decision variables
  Xin = {q_1, q_2}; joint angles

bounds
  q_1, q_2 = {-3.14, 3.14}

*/


/******************************/
void simulator(double Xin[2],double Xout[2])
{
  dsim->qpos[0] = Xin[0]; dsim->qpos[1] = Xin[1];
  dsim->ctrl[0] = dsim->qpos[0]; dsim->ctrl[2] = dsim->qpos[1];
  mj_forward(m,dsim);
  Xout[0] = dsim->sensordata[0];
  Xout[1] = dsim->sensordata[2];
  // printf("tip: %f %f \n",d->sensordata[0], d->sensordata[2]);
}
//**************************

// typedef struct {
//     double a[5];
// }mycost_data;
//
// typedef struct {
//     double ceq_1;
//     double ceq_2;
// }myequalityconstraints_data;
//
// typedef struct {
//     double cin_1;
// }myinequalityconstraints_data;


double mycost(unsigned n, const double *x, double *grad, void *costdata)
{
    // mycost_data *data = (mycost_data *) costdata;
    //
    // int i;
    // double a[5]={0};
    // for (i=0;i<n;i++)
    //   a[i] = data->a[i];

    double cost = 0;
    // for (i=0;i<n;i++)
    //   cost += a[i]*x[i]*x[i];

    return cost;
}


double myequalityconstraints(unsigned m, double *result, unsigned n,
                             const double *x,  double *grad,
                             void *equalitydata)
{
    // myequalityconstraints_data *data = (myequalityconstraints_data *) equalitydata;
    //
    // double c1 = data->ceq_1;
    // double c2 = data->ceq_2;
    // double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];
    // result[0] = x1+x2+x3-c1; //5;
    // result[1] = x3*x3+x4-c2; //2;


    // equality constraints:
    //   ceq_1 = x_target - sensordata[0];
    //   ceq_2 = z_target - sensordata[2];
    double Xout[2]={0};
    simulator(x,Xout);

    result[0] = x_target - Xout[0]; //d->sensordata[0];
    result[1] = z_target - Xout[1]; //d->sensordata[2];
 }

 // double myinequalityconstraints(unsigned m, double *result, unsigned n,
 //                                const double *x,  double *grad,
 //                                void* inequalitydata)
 // {
 //     myinequalityconstraints_data *data = (myinequalityconstraints_data *) inequalitydata;
 //
 //     double c1 = data->cin_1;
 //     double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];
 //     result[0] = x4*x4+x5*x5-c1; //5;
 //  }


void inverse_kinematics(double Xin[2], double Xref[2])
{
int i;
nlopt_opt opt;

x_target = Xref[0];
z_target = Xref[1];

//double Xin[2]={-0.5,1}; //joint angle
//double Xout[2] = {0}; //tip position
//simulator(Xin,Xout);
//printf("tip: %f %f \n",Xout[0], Xout[1]);

//establish sizes
unsigned n = 2; //number of decision variables
unsigned m_eq = 2; //number of equality constraints
//unsigned m_in = 1; //number of inequality constraints

//bounds for decision variables
double lb[] = { -3.14, -3.14 }; /* lower bounds */
double ub[] = { 3.14, 3.14 }; /* lower bounds */

//Set the algorithm and dimensionality
//L,G = global/local
//D,N = derivative / no derivative
opt = nlopt_create(NLOPT_LN_COBYLA, n); /* algorithm and dimensionality */

//Set the lower and upper bounds
nlopt_set_lower_bounds(opt, lb);
nlopt_set_upper_bounds(opt, ub);

//Set up cost
// mycost_data costdata;
// for (i=0;i<n;i++)
//   costdata.a[i]=1;
nlopt_set_min_objective(opt, mycost, NULL);

//set up equality constraint
double tol_eq[]={1e-8,1e-8};
// myequalityconstraints_data equalitydata;
// equalitydata.ceq_1 = 5;
// equalitydata.ceq_2 = 2;
nlopt_add_equality_mconstraint(opt, m_eq, myequalityconstraints, NULL, tol_eq);

// double tol_in[]={1e-8};
// myinequalityconstraints_data inequalitydata;
// inequalitydata.cin_1 = 5;
// nlopt_add_inequality_mconstraint(opt, m_in, myinequalityconstraints,&inequalitydata, tol_in);


nlopt_set_xtol_rel(opt, 1e-4);
//double x[] = { 1, 1, 1, 2, 1 };  // initial guess
double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
if (nlopt_optimize(opt, Xin, &minf) < 0) {
    printf("nlopt failed!\n");
}
else {
    printf("found minimum at f(%g,%g) = %0.10g\n", Xin[0], Xin[1],minf);
}

nlopt_destroy(opt);
}
