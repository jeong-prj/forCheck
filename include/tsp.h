#ifndef TSP_H
#define TSP_H

extern "C" {
  #include <concorde.h>
}

class TSP {
 public:
  struct TSPSolver {
    int ncount;
    int *out_tour = (int *) NULL;;

    double optval; //Value of the optimal tour
    double *in_val = (double *) NULL; //Can be used to specify an initial upperbound (it can be NULL)
    int *in_tour = (int *) NULL; //Gives a starting tour in node node node format (it can be NULL)
    char *name = (char *) NULL; //Specifes a char string that will be used to name various files that are written during the branch and bound search
    double timebound_ = 100;
    double *timebound = &timebound_; //Run time limit
    int success; //1 if the run finished normally, and set to 0 if the search was terminated early (by hitting some predefined limit)
    int foundtour; //1 if a tour has been found (if success is 0, then it may not be the optimal tour)
    int hit_timebound = 0; //1 if timebound was reached
    int silent = 1; //Suppress most output if set to a nonzero value
  };
};
#endif  // TSP_H
