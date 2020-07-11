import numpy as np
import pdb
import datetime
from Utilities import Curvature
from DynamicsModel import DynModel

# TODO: modify this only for MPC (remove the contents for LMPC)

class Simulator():
    """Vehicle simulator
    Attributes:
        Sim: given a Controller object run closed-loop simulation and write the data in the ClosedLoopData object
    """
    def __init__(self, map, lap = 0, flagLMPC = 0):
        """Initialization
        map: map
        lap: number of laps to run. If set to 0 then the simulation is completed when ClosedLoopData is full
        flagLMPC: set to 0 for standart controller. Set to 1 for LMPC --> at iteration j add data to SS^{j-1} (look line 9999)
        """
        self.map = map
        self.laps = lap
        self.flagLMPC = flagLMPC

    def Sim(self, ClosedLoopData, Controller, LMPCprediction=0):
        """Simulate closed-loop system
        ClosedLoopData: object where the closed-loop data are written
        Controller: controller used in the closed-loop
        LMPCprediction: object where the open-loop predictions and safe set are stored
        """

        # Assign x = ClosedLoopData.x. IMPORTANT: x and ClosedLoopData.x share the same memory and therefore
        # if you modofy one is like modifying the other one!
        x      = ClosedLoopData.x
        x_glob = ClosedLoopData.x_glob
        u      = ClosedLoopData.u

        SimulationTime = 0
        for i in range(0, ClosedLoopData.Points):

            Controller.solve(x[i, :])

            u[i, :] = Controller.uPred[0,:]

            if LMPCprediction != 0:
                LMPCprediction.PredictedStates[:, :, i, Controller.it]   = Controller.xPred
                LMPCprediction.PredictedInputs[:, :, i, Controller.it] = Controller.uPred
                LMPCprediction.SSused[:, :, i, Controller.it]          = Controller.SS_PointSelectedTot
                LMPCprediction.Qfunused[:, i, Controller.it]           = Controller.Qfun_SelectedTot

            x[i + 1, :], x_glob[i + 1, :] = DynModel(x[i, :], x_glob[i, :], u[i, :], np, ClosedLoopData.dt, self.map.PointAndTangent)
            SimulationTime = i + 1

            if i <= 5:
                print("Linearization time: %.4fs Solver time: %.4fs" % (Controller.linearizationTime.total_seconds(), Controller.solverTime.total_seconds()))
                print("Time: ", i * ClosedLoopData.dt, "Current State and Input: ", x[i, :], u[i, :])

            if Controller.feasible == 0:
                print("Unfeasible at time ", i*ClosedLoopData.dt)
                print("Cur State: ", x[i, :], "Iteration ", Controller.it)
                break

            if self.flagLMPC == 1:
                Controller.addPoint(x[i, :], u[i, :], i)

            if (self.laps == 1) and (int(np.floor(x[i+1, 4] / (self.map.TrackLength))))>0:
                print("Simulation terminated: Lap completed")
                break

        ClosedLoopData.SimTime = SimulationTime
        print("Number of laps completed: ", int(np.floor(x[-1, 4] / (self.map.TrackLength))))