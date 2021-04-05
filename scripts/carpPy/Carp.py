#!/usr/bin/env python
import time
# math
import numpy as np
import numpy.linalg as la
import numpy.random as rand
import cvxpy as cvx


class Carp(object):
    """docstring for Carp"""

    def __init__(self):
        self.objective = 0

    def project(self, position, goal, obstacles):
        """GAA's magical wizardy for the original problem (projection)"""
        # get constants
        p = position  # p
        g = goal  # g goal point

        # create cvx program
        n = len(p)
        x = cvx.Variable(n)  # projection pt
        t = cvx.Variable(n)  # slack
        L = cvx.Variable(len(obstacles), nonneg=True)  # dual on elip
        # form objective.
        self.objective = cvx.norm(x - g, 2)
        self.constraints = []
        for i, (center, shape) in enumerate(obstacles):
            # cnts += [L[i] >= 0]
            S = shape.reshape((n, n))  # sigma
            S_inv = la.inv(S)  # sigma inv
            u = center  # mu
            # get eigdecomp
            D, U = la.eig(S)

            # constraints
            Q = np.dot(U.T, S_inv.dot(u))
            LHS = 0
            for d in range(n):
                LHS += cvx.quad_over_lin(L[i]*Q[d] + U.T[d, :]*x,
                                         L[i]*(D[d]**-1) + 1)
            self.constraints += [LHS - 2*p.T*x
                                 <= L[i] * np.dot(u, S_inv.dot(u)) - cvx.sum_squares(p)-L[i]]

        # Form and solve problem.
        prob = cvx.Problem(cvx.Minimize(self.objective), self.constraints)
        tic = time.time()
        prob.solve(solver=cvx.ECOS, reltol=1e-3, abstol=1e-3)
        print("Projection solution time {:2.4f}sec".format(time.time()-tic))
        if x.value is None:
            raise RuntimeError("failed to project")
        return np.array(x.value).flatten(), self.objective.value

    def projectpoly(self, v=None, aMax=1., order=3):
        """GAA's magical wizardy for the original problem (projection)
        now with polynomials"""
        # get constants
        g = self.goal  # g goal point
        p = self.position  # p
        # get dim, either 2 or 3
        n = len(p)
        if v is None:
            v = np.zeros(n)

        goalDist = self.goalDist()
        minDist = min([ob.estimate.dist(g) for ob in self.obstacles])
        if minDist > goalDist:
            return np.vstack((p, v/3.+p, g, g)).T, goalDist

        # create cvx program
        x = cvx.Variable((n, order+1))  # projection pt
        L = cvx.Variable(len(self.obstacles),
                         nonneg=True)  # dual on elip
        obj = cvx.Minimize(cvx.norm(x[:, -1] - g, 2))
        cnts = []
        for i, ob in enumerate(self.obstacles):
            estimate = ob.estimate
            # check if we need to project
            if ob.estimate.dist(g) > goalDist:
                cnts += [L[i] == 0]
            else:
                cnts += [L[i] >= 0]
                u = estimate.center  # mu
                S = estimate.body  # sigma
                S_inv = la.inv(S)  # sigma inv
                D, U = la.eig(S)   # get eigdecomp

                # Form objective.
                Q = np.dot(U.T, S_inv.dot(u))
                for j in range(order+1):
                    cnts += [np.sum([
                        cvx.quad_over_lin(L[i]*Q[d] + U.T[d, :]*x[:, j],
                                          L[i]*(D[d]**-1) + 1)
                        for d in range(n)])
                        - 2*p.T*x[:, j]
                        <= L[i] * np.dot(u, S_inv.dot(u))
                        - cvx.sum_squares(p)-L[i]]
        # dynamics on polynomial
        cnts += [x[:, 0] == p]  # ic pos
        cnts += [3*(x[:, 1] - x[:, 0]) == v]  # ic vel
        cnts += [cvx.norm(6*(x[:, 2]-2*x[:, 1]+x[:, 0]), 2) <= aMax]  # ic acel

        cnts += [3*(x[:, -2] - x[:, -1]) == 0]  # final vel
        cnts += [cvx.norm(6*(x[:, 3]-2*x[:, 2]+x[:, 1]), 2)
                 <= aMax]  # final ace

        # Form and solve problem.
        prob = cvx.Problem(obj, cnts)

        # tic = time.time()
        prob.solve(solver=cvx.ECOS, reltol=1e-3, abstol=1e-3)
        # print(prob.status)
        # print(f"Projection solution time {time.time()-tic:2.4f}sec")
        return np.array(x.value), obj.value


if __name__ == '__main__':
    carp = Carp()

    # test data
    pos = np.array([2, 2, 2])
    goal = np.array([0, 0, 0])
    center = np.array([0, 1, 0])
    shape = np.eye(3).flatten()
    obs = [(center, shape)]

    projection, ob = carp.project(pos, goal, obs)
