#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

using namespace std;
using namespace SESync;

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Usage: " << argv[0] << " [input .g2o file]" << endl;
    exit(1);
  }

  size_t num_poses;
  measurements_t measurements = read_g2o_file(argv[1], num_poses);
  if (measurements.size() == 0) {
    cout << "Error" << endl;
    exit(1);
  }

  SESyncOpts opts;
  opts.verbose = false; // Don't print output to stdout
  opts.num_threads = 4;
  // opts.preconditioned_grad_norm_tol = 1e-6;

  /// RUN SE-SYNC!
  SESyncResult results = SESync::SESync(measurements, opts);

  // Write output
  std::setprecision(std::numeric_limits<long double>::digits10 + 1);
  std::cout << "{" << std::endl;
  /** The value of the rounded solution xhat in SE(d)^n */
  std::cout << "\"f_hat\":" << results.Fxhat << "," << std::endl;
  /** The value of the objective F(Y^T Y) = F(Z) attained by the Yopt */
  std::cout << "\"f_sdp\":" << results.SDPval << "," << std::endl;
  std::cout << "\"x_hat_shape\": [" << results.xhat.rows() << "," << results.xhat .cols() << "]," << std::endl;
  std::cout << "\"x_hat\":\"" << results.xhat << "\"" << std::endl;
  std::cout << "}" << std::endl;
}
