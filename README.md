# G4ILSCS

This repository contains the problem instances and the heuristic implementations used in the paper:

Hybrid heuristics for a 4-level integrated lot-sizing and cutting stock problem with multiple suppliers

Repository Contents

- Problem instances used in the computational experiments:\
  NumS_ucK_vwFK__Class1-9\
  NumS_ucK_vwFK__Class10-18\
  NumS_ucK_vwFK__Class19-27\
  Note: see the file named 'NameClasses' inside the folders for more details about each class and data variation.

- Implementation of the heuristic solution approaches used in the paper:\
  G4ILSCS_HH_F: Hybrid Heuristic with product-oriented decomposition\
  G4ILSCS_HH_T: Hybrid Heuristic with time-oriented decomposition\
  G4ILSCS_HH_L123: Hybrid Heuristic with level-oriented decomposition (levels 123)\
  G4ILSCS_HH_L231: Hybrid Heuristic with level-oriented decomposition (levels 231)\
  G4ILSCS_CGH: Benchmark Heuristic\
  G4ILSCS_CGH: Sequential Heuristic of levels\

Paper Abstract:\
In this paper, we propose a generalized 4-level integrated lot-sizing and cutting stock problem that addresses key decisions within the supply chain, including the availability of multiple suppliers for purchasing raw materials that will be cut into small pieces, and the transportation of final products to a warehouse. To solve this integrated problem, we propose hybrid heuristics designed to overcome its inherent complexities. The hybrid methods incorporate two decomposition approaches within each iteration: a column generation procedure and a relax-and-fix procedure. Given the specific characteristics of the problem, we propose an innovative column generation approach to tackle cutting patterns in the cutting stock problem and cargo configurations in the transportation problem. First, the performance of the integrated approach is compared to a sequential approach which is commonly used in practice, showing the benefits of integrating processes in an industrial setting. Next, the proposed methods are employed to assess the impact of integrating these decisions across the supply chain, and the performance is compared against a benchmark heuristic from the literature to assess their effectiveness in solving this integrated problem.

