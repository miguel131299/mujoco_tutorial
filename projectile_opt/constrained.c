#include <stdio.h>
#include <math.h>

#include "nlopt.h"

double x_target = 5;
double z_target = 2.1;

double x_end;
double z_end;

extern mjModel* m;                  // MuJoCo model
extern mjData* d;

void simulator(double Xin[3],double Xout[2])
{
  double v,theta,time_of_flight;
  v = Xin[0];
  theta = Xin[1];
  time_of_flight = Xin[2];

  d->qvel[0] = v*cos(theta);
  d->qvel[2] = v*sin(theta);
  while (d->time < time_of_flight)
  {
    mj_step(m,d);
  }
  //printf("%f %f %f \n",d->time,d->qpos[0],d->qpos[2]);
  Xout[0] = d->qpos[0];
  Xout[1] = d->qpos[2];
  mj_resetData(m,d);
}


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


double mycost(unsigned n, double *x, double *grad, void *costdata)
{
    //mycost_data *data = (mycost_data *) costdata;

    // int i;
    // double a[5]={0};
    // for (i=0;i<n;i++)
    //   a[i] = data->a[i];
    double Xout[2]={0};
    simulator(x,Xout);
    x_end = Xout[0];
    z_end= Xout[1];

    double cost = 0*x[2];
    // for (i=0;i<n;i++)
    //   cost += a[i]*x[i]*x[i];

    return cost;
}


double myequalityconstraints(unsigned m, double *result, unsigned n,
                             const double *x,  double *grad,
                             void *equalitydata)
{
    //myequalityconstraints_data *data = (myequalityconstraints_data *) equalitydata;

    // double c1 = data->ceq_1;
    // double c2 = data->ceq_2;
    //double x1 = x[0], x2 = x[1], x3 = x[2], x4 = x[3], x5 = x[4];

    //void simulator(double Xin[3],double Xout[2])

    //double x_end = Xout[0], z_end=Xout[1];
    result[0] = x_end-x_target; //5;
    result[1] = z_end-z_target; //2;
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


void optimize_ic(double Xin[3])
{
int i;
nlopt_opt opt;

//establish sizes
unsigned n = 3; //number of decision variables
unsigned m_eq = 2; //number of equality constraints
//unsigned m_in = 1; //number of inequality constraints

//bounds for decision variables
double lb[] = { 0.1, 0.1, 0.1 }; /* lower bounds */
double ub[] = { HUGE_VAL, 3.14/2-0.1, HUGE_VAL }; /* lower bounds */

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
double tol_eq[]={1e-4,1e-4};
// myequalityconstraints_data equalitydata;
// equalitydata.ceq_1 = 5;
// equalitydata.ceq_2 = 2;
nlopt_add_equality_mconstraint(opt, m_eq, (nlopt_mfunc)myequalityconstraints, NULL, tol_eq);

// double tol_in[]={1e-8};
// myinequalityconstraints_data inequalitydata;
// inequalitydata.cin_1 = 5;
// nlopt_add_inequality_mconstraint(opt, m_in, myinequalityconstraints,&inequalitydata, tol_in);
//

nlopt_set_xtol_rel(opt, 1e-4);
//double x[] = { 1, 1, 1, 2, 1 };  // initial guess
double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
if (nlopt_optimize(opt, Xin, &minf) < 0) {
    printf("nlopt failed!\n");
}
else {
    printf("found minimum at f(%g,%g,%g) = %0.10g\n", Xin[0], Xin[1],Xin[2], minf);
}

nlopt_destroy(opt);
}
