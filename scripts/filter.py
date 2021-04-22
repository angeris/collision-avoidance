#!/bin/bash/python
# std
import warnings as warn
import os as os
import time
# math
import numpy as np
import numpy.linalg as la
import numpy.random as rand
from scipy.linalg import qr
# import cvxpy as cvx

# plot
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d

# from utils import *


class LinearModel(object):
    def __init__(self, A, B, C, D=0):
        self.A = A
        self.B = B
        self.C = C


class FilterBounded(object):
    def __init__(self, center, body,
                 processNoise, measurementNoise,
                 model='basic', initNoise=True):
        self.dim = len(center)
        center = np.array(center)
        # norm ball with size equal to the agents max speed
        self.pNoise = processNoise
        self.mNoise = measurementNoise
        if model == 'basic':
            self.model = LinearModel(
                np.eye(self.dim), np.eye(self.dim), np.eye(self.dim))
        else:
            self.model = model
        self.center = self.measure(center) if initNoise else center  # mean
        self.body = body  # covariance
        self.scale = 1.0  # scalar for the body metric, used by the update algo

        # defaults
        self.thetaBounds_default = [-np.pi, np.pi]
        self.phiBounds_L_default = -np.pi/2

        # cache
        self.G = la.cholesky(self.body)

    def measure(self, truePosition, noise=True):
        # returns a sample drawn from the bounded set v'Sm^-1v<1
        # where Sm is self.mNoise

        # rejection sample
        if not noise:
            return truePosition
        pt = rand.random(self.dim)*2-1  # sample from square
        while la.norm(pt) > 1:  # rejection
            pt = rand.random(self.dim)*2-1  # sample from square
        F = la.cholesky(self.mNoise)
        return F.dot(pt)+truePosition

    def getAxisLens(self, S):
        # semi axis for the following elipsoid
        # e | (e-c)^T S^-1(e-c)=1
        # print self.S
        L, T = la.eig(S)
        # print T
        S = np.array([np.power(li, .5) for li, ti in zip(L, T)])
        return S, T

    def projectEstimate(self, S=None):
        # return a projected estimate object that takes into future size
        # nSteps is the amount of time in the future without measurement update

        A = self.model.A
        B = self.model.B
        C = self.model.C

        center_hat = self.center
        if S is None:
            body_hat = self.body
        else:
            body_hat = S
        # minimal trace method
        p = np.sqrt(np.trace(A.dot(body_hat.dot(A.T)))
                    ) + np.sqrt(np.trace(self.pNoise))
        p = np.sqrt(np.trace(self.pNoise))/p

        center_hat = np.dot(A, center_hat)
        # itermediate body matrix
        body_hat = np.power(1-p, -1)*A.dot(body_hat.dot(A.T)) + \
            np.power(p, -1) * self.pNoise

        return body_hat

        # e | (e-c)^T S^-1(e-c)=1

    def project(self, pt):
        # projects a pt onto the ellipsoid
        n = len(pt)
        assert n**2 == len(self.body.flatten())
        x = cvx.Variable(n)

        # Form objective.
        obj = cvx.Minimize(cvx.norm(x - pt, 2))

        cnts = [cvx.quad_form(x-self.center, la.inv(self.body)) <= 1]
        # Form and solve problem.
        prob = cvx.Problem(obj, cnts)
        prob.solve()
        return np.array(x.value).flatten()

    def addMargin(self, r, typ='ball'):
        ''' returns an the body of an ellipsoid that contains the estimate
        with a margin of at least r '''
        # outer bounding ellipsoid of the minsowski sum of epi and radius r ball
        if typ == 'ball':
            if not isinstance(r, float):
                raise RuntimeError("margin type doesn't match argument")
            margin = np.eye(self.dim)
            margin *= r**2
        if typ == 'elip':
            margin = r

        p = np.sqrt(np.trace(self.scale*self.body)) + np.sqrt(np.trace(margin))
        p = np.sqrt(np.trace(margin))/p

        # intermediate body matrix
        return np.power(1-p, -1)*self.body + np.power(self.scale*p, -1)*margin

    def dist(self, pt):
        # finds the distance between the point pt and the elip
        return la.norm(pt - self.project(pt))

    def elipMap(self, d):
        # maps a point d on the unit sphere to the ellipsoid defined with self
        return self.G.dot(d)+self.center

    def normalMap(self, d):
        # maps a point d on the unit sphere to it's normal on the ellipsoid
        return np.dot(la.inv(self.G.T), d)

    def update(self, measurement):
        # implements some sort of bounded set measurement update
        # method from Yushuang Liu ,Yan Zhao, Falin Wu
        # unpack
        self.scale = 1
        A = self.model.A
        B = self.model.B
        C = self.model.C
        # time update

        # mixing parameter for time
        # minimal trace method
        p = np.sqrt(np.trace(self.scale*A.dot(self.body.dot(A.T)))) + \
            np.sqrt(np.trace(self.pNoise))
        p = np.sqrt(np.trace(self.pNoise))/p

        center_hat = np.dot(A, self.center)
        body_hat = np.power(1-p, -1)*A.dot(self.body.dot(A.T)) + \
            np.power(self.scale*p, -1)*self.pNoise  # intermediate body matrix

        # measurement update
        # upper bound method
        delta = measurement-C.dot(center_hat)  # innovation
        CP_quad = C.dot(body_hat.dot(C.T))  # C*P_hat*C.T
        V_bar = la.cholesky(la.inv(self.mNoise)).T
        delta_bar = V_bar.dot(delta)
        G = V_bar.dot(CP_quad.dot(V_bar.T))
        val, vec = la.eig(G)
        g = max(val)

        beta = (1.0-self.scale)/la.norm(delta_bar)
        # mixing parameter for measurement
        if self.scale+la.norm(delta_bar) <= 1.0:
            L = 0.0
        elif g == 1:
            L = (1.0-beta)/2.0
        else:
            L = 1.0/(1.0-g) * (1.-np.sqrt(g/(1.+beta*(g-1.))))
        # print l
        if L == 0:
            # handle the edge case
            Q_inv = np.zeros((self.dim, self.dim))
        else:
            Q = np.power(L, -1)*self.mNoise+np.power(1-L, -1) * \
                CP_quad  # intermediate thing
            Q_inv = la.inv(Q)

        K = np.power(1-L, -1)*body_hat.dot(np.dot(C.T, Q_inv))  # "kalman" gain
        self.scale = (1-L)*self.scale+L-np.dot(delta.T,
                                               Q_inv.dot(delta))  # scale
        # print("scale dec: ", self.scale)
        self.body = self.scale * \
            np.power(1-L, -1)*np.dot(np.eye(self.dim)-K.dot(C), body_hat)
        self.center = center_hat+K.dot(delta)
        self.G = la.cholesky(self.body)


class FilterBounded3D(FilterBounded):
    """docstring for EstimateBounded3D
    implements the 3D version of the abstract bounded estimation class"""

    def __init__(self, *args, **kwargs):
        super(FilterBounded3D, self).__init__(*args, **kwargs)

        # defaults
        self.thetaBounds_default = [-np.pi, np.pi]
        self.phiBounds_L_default = -np.pi/2

    def sphereCord(self, theta, phi):
        # returns pt or pts on the sphere with radius 1
        return np.array([np.cos(theta)*np.cos(phi),
                         np.sin(theta)*np.cos(phi),
                         np.sin(phi)])

    def getRefAngle(self, ePt):
        # calculates the ref arguments (theta, phi) of a point e on the elip,
        # in the world frame

        # check if pt is on elip
        S_inv = la.inv(self.body)
        u = self.center
        z = ePt-u
        err = abs(np.dot(z, S_inv.dot(z)) - 1)
        if abs(err) > 1e-4:
            print("point is not on elipsoid")

        F = la.cholesky((self.body))
        d = np.dot(la.inv(F), ePt-self.center)

        # get phi
        phi = np.arcsin(d[2])
        # get theta
        theta = np.arctan2(d[1]/np.cos(phi), d[0]/np.cos(phi))
        return theta, phi

    def elips(self,
              thetaRange=None, phiBounds_L=None, RI=np.eye(3),
              pts=20, check=False):
        # e | (e-c)^T S^-1(e-c)=1

        # set up ranges and bounds
        if thetaRange is None:
            thetaRange = np.linspace(
                self.thetaBounds_default[0], self.thetaBounds_default[1], pts)

        if phiBounds_L is None:
            phiBounds_L = np.array([self.phiBounds_L_default]*pts)
        if isinstance(phiBounds_L, float):
            phiBounds_L = np.array([phiBounds_L]*pts)

        # build object
        elip = np.zeros((self.dim, pts, pts))
        for i, theta in enumerate(thetaRange):
            phiRange = np.linspace(phiBounds_L[i], np.pi/2, pts)
            for j, phi in enumerate(phiRange):
                d = self.sphereCord(theta, phi)
                elip[:, i, j] = self.elipMap(RI.dot(d))

        # print elip.shape
        if check:
            if not self.elipCheck(elip):
                warn.warn("elipsoid does not match")

        return elip

    def normals(self,
                thetaRange=None, phiBounds_L=None, RI=np.eye(3),
                pts=20, check=False):
        # e | (e-c)^T S^-1(e-c)=1

        # set up ranges and bounds
        if thetaRange is None:
            thetaRange = np.linspace(
                self.thetaBounds_default[0], self.thetaBounds_default[1], pts)

        if phiBounds_L is None:
            phiBounds_L = np.array([self.phiBounds_L_default]*pts)
        if isinstance(phiBounds_L, float):
            phiBounds_L = np.array([phiBounds_L]*pts)

        if len(thetaRange) != len(phiBounds_L):
            raise ValueError("range length mismatch")

        # build object
        normals = np.zeros((self.dim, pts, pts))
        for i, theta in enumerate(thetaRange):
            phiRange = np.linspace(phiBounds_L[i], np.pi/2, pts)
            for j, phi in enumerate(phiRange):
                d = self.sphereCord(theta, phi)
                normals[:, i, j] = self.normalMap(RI.dot(d))

        return normals

    def elipCheck(self, elip):
        # checks if elip is in ellipsoid defined by  (e-u)^T S^-1(e-u)=1
        S_inv = la.inv(self.body)
        u = self.center
        for j in range(elip.shape[2]):
            for i in range(elip.shape[1]):
                z = elip[:, i, j]-u
                err = abs(np.dot(z, S_inv.dot(z)) - 1)
                if err > 10e-7:
                    print("failed elip check at index", i, j)
                    return False
        return True

    def plot(self, ax, color='r', alpha=.5, pts=20):
        elips = self.elips(pts=pts)
        # ax.plot([self.center[0]], [self.center[1]], [self.center[2]],
        #  marker='^',marker size=2, color=color)
        ax.plot_surface(elips[0, :, :], elips[1, :, :], elips[2, :, :],
                        color=color, alpha=alpha,
                        rstride=1, cstride=1,  antialiased=True)


if __name__ == '__main__':

    # test bounded 3D
    u_true = np.array([.02, .4, .1])
    u = np.array([0.0, 0.0, 0.0])
    # S=np.array([[2, .50, .50],[.50, 1.0, .2 ], [.5, .2, 1.0]])

    # S=np.array([[1, 0, 0],[0, 2, .0 ], [0, 0, 2.0]])
    S = np.array([[2, .50, .50], [.50, 1.0, .2], [.5, .2, 1.0]])
    # S=np.eye(3)*4
    # S=np.eye(3)
    noise = np.eye(3)
    pts = 20
    estimate = FilterBounded3D(u, S, noise, noise*.5)

    margin = estimate.addMargin(1.)
    print(estimate.body)
    print(margin)
    # testPt = np.array([1, .8, .8])
    # testPtm = np.array([1, .8, .8]) + np.array([1, .0, .0])
    # print np.dot(testPt.T, la.inv(estimate.body).dot(testPt))
    # print np.dot(testPtm.T, la.inv(margin).dot(testPtm))
    elip = estimate.elips(pts=pts)
    normals = estimate.normals(pts=pts)
    # print elip.shape
    # print normals.shape

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    ax.plot([u_true[0]], [u_true[1]], [u_true[2]], marker='o', c='k')

    # ax.plot(elip[0,:,:].flatten(), elip[1,:,:].flatten(),
    #         elip[2,:,:].flatten(), color='m', marker='o')
    equiver = np.reshape(elip, (3, pts**2))
    nquiver = np.reshape(normals, (3, pts**2))
    # ax.plot(equiver[0,:], equiver[1,:], equiver[2,:],
    #         color='k', linestyle=':' )

    # test 2 project
    # u_true=np.array([.89, 1.4]);
    # u=np.array([1.0,1.0]);
    # S=np.array([[2, .50],[.5, 1.]])
    # S=np.eye(2)
    # noise=np.eye(2)*.2
    # evaderModel=LinearModel(np.eye(2), np.eye(2), np.eye(2))
    # estimate=EstimateBounded(evaderModel, u, S, noise, noise*.5)
    # pts=40
    # elip= estimate.Elips2D(pts=pts)
#     # fig = plt.figure(); ax = fig.add_subplot(1,1,1)
    # ax.plot([u_true[0]],[u_true[1]], marker='o', c='r')
    # ax.plot(elip[:,0], elip[:,1],color='m', alpha=.1, )
    # estimateProj= estimate.projectEstimate(5)
    # elipProj= estimateProj.Elips2D(pts=pts)
    # ax.plot(elipProj[:,0], elipProj[:,1], color='g', alpha=.1)

# #test filter
    color_list = plt.cm.Set1(np.linspace(0, 1, 9))
    for i in range(8):
        # u_true moves
        u_true += np.array([.4, -.4, .7])
#         #measurement
        u_measure = estimate.measure(u_true)
        # print u_measure
        ax.plot([u_true[0]], [u_true[1]], [u_true[2]],
                marker='o', c=color_list[i])  # new true possition
        ax.plot([u_measure[0]], [u_measure[1]], [u_measure[2]],
                marker='^', c=color_list[i])  # measurement of new position
#         #do update
        s = time.time()
        estimate.update(u_measure)
        print(time.time()-s)
        estimate.plot(ax, color=color_list[i], alpha=.1)
    plt.axis("off")

    W = [[-2, 2], [-2, 2], [-2, 2]]
    ax.set_xlim(*W[0])
    ax.set_ylim(*W[1]), ax.set_zlim(*W[2])
    # ax.axis("equal")
    plt.show()
