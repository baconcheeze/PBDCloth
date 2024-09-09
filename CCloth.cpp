#include "pch.h"

#include "MPMSetting.h"
#include "MPMAlgebra.h"
#include "CTimeMgr.h"

#include "CCloth.h"

float Gravity = 98;

#define DT CTimeMgr::GetInst()->dt()

namespace PBD
{
	using namespace MPMSDF;

	CCloth::CCloth()
	{

	}
	CCloth::~CCloth()
	{
	}

	void CCloth::init()
	{
		N = 100;

		mPos.resize(N);
		pPos.resize(mPos.size());
		mVel.resize(mPos.size());
		mInvMass.resize(mPos.size());
		mCollisionFlag.resize(N);

		mPos[0].setData(100, 150);
		mPos[N-1].setData(200, 150);
		mInvMass[0] = 0;
		mInvMass[N-1] = 0;

		mBall.resize(1);
		pBall.resize(mBall.size());
		mBallVel.resize(mBall.size());
		mBallInvMass.resize(mBall.size());

		mBall[0].setData(170, 210);
		mBallInvMass[0] = 0.1;

		for (int i = 1; i < N-1; ++i)
		{
			mPos[i].setData(100+100.f * i / (N - 1), 150);		
			mInvMass[i] = 1;
		}

		
		int a = 0;
	}

	void CCloth::tick()
	{
		float dt = DT;

		for (int i=0;i<mPos.size();++i)
		{
			mVel[i].val[1] -= Gravity * DT;
			pPos[i] = mVel[i] * DT + mPos[i];	

			//mPos[N - 1][0] -= DT * 1;
		}

		for (int i = 0; i < mBall.size(); ++i)
		{
			mBallVel[i].val[1] -= Gravity * DT;
			pBall[i] = mBallVel[i] * DT + mBall[i];

			//mPos[N - 1][0] -= DT * 1;
		}

		for (int i = 0; i < N; ++i)
		{
			if ((pPos[i] - pBall[0]).norm() < 10)
				mCollisionFlag[i] = true;

			else
				mCollisionFlag[i] = false;
		}

		//PBD
		
		int iterationcount = 10;

		for (int n = 0; n < iterationcount; ++n)
		{
			pPos[0] = mPos[0];
			pPos[N-1] = mPos[N-1];			

			for (int i = 0; i < N; ++i)
			{
				if(mCollisionFlag[i])
					DistanceConstraint(pPos[i], pBall[0], mInvMass[i], mBallInvMass[0], 10, 0.1, iterationcount);
			}

			for (int i = 0; i < N - 1; ++i)
			{
				DistanceConstraint(pPos[i], pPos[i + 1], mInvMass[i], mInvMass[i + 1], 100.f / (N - 1), 1.0, iterationcount);
			}
				
		}

		//

		for (int i = 0; i < mPos.size(); ++i)
		{
			mVel[i] = (pPos[i] - mPos[i]) / DT;
			mPos[i] = pPos[i];
		}

		for (int i = 0; i < mBall.size(); ++i)
		{
			mBallVel[i] = (pBall[i] - mBall[i]) / DT;
			mBall[i] = pBall[i];
		}
	}
	
	void DistanceConstraint(MPMSDF::Vector2f& p0, MPMSDF::Vector2f& p1, float w0, float w1, float d,float k,int N)
	{
		float D = (p0 - p1).norm();
		float kprime = 1 - pow(1 - k, N);

		MPMSDF::Vector2f delta0 = -w0 / (w0 + w1) * (D - d) * (p0 - p1) / D;
		MPMSDF::Vector2f delta1 = w1 / (w0 + w1) * (D - d) * (p0 - p1) / D;

		p0 += kprime * delta0;
		p1 += kprime * delta1;
	}
}