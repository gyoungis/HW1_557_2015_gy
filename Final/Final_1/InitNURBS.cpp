#include "NURBS.h"
#pragma warning(push)
#pragma warning(disable : 4305 4244)

void NURBS::InitNURBS(int surfNum)
{
	float* ctlpoints;
	float* uKnots;
	float* vKnots;
	if (surfNum==1)
	{
		this->trimmed=false;
		this->uBaseNum=15;
		this->vBaseNum=30;

		this->kdColor = Float3(0.760784, 0.843137, 0.196078);
		this->ksColor = Float3(0.9, 0.9, 0.9);
		this->ka = 0.1;
		this->shininess = 50;

		float weights[14][13]={1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.5,1,1,1,1,1,0.5,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

		float tempctlpoints[14][13][3]={
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.92379,0.0131926,-0.535702,
			1.92379,0.0465361,-0.535702,
			1.90747,0.121179,-0.535702,
			1.57358,0.236742,-0.535702,
			1.41966,0.237449,-0.535702,
			1.30075,0.120118,-0.535702,
			1.27627,0.0131926,-0.535702,
			1.30075,-0.093733,-0.535702,
			1.41966,-0.211064,-0.535702,
			1.57358,-0.210357,-0.535702,
			1.90747,-0.0947938,-0.535702,
			1.92379,-0.0201509,-0.535702,
			1.92379,0.0131926,-0.535702,
			2.73528,0.0131926,-0.516333,
			2.73528,0.118624,-0.516333,
			2.6288,0.354652,-0.516333,
			1.80454,0.720067,-0.516333,
			1.00825,1.13359,-0.516333,
			0.455347,0.351297,-0.516333,
			0.295617,0.0131926,-0.516333,
			0.455347,-0.324912,-0.516333,
			1.00825,-1.1072,-0.516333,
			1.80454,-0.693681,-0.516333,
			2.6288,-0.328267,-0.516333,
			2.73528,-0.092239,-0.516333,
			2.73528,0.0131926,-0.516333,
			3.43266,0.0131926,0.450235,
			3.43266,0.194338,0.450235,
			3.06333,1.15022,0.450235,
			2.5118,0.943504,-0.0916114,
			1.21331,1.45996,-0.0657666,
			-0.0331817,0.756754,-0.0657666,
			-0.225645,0.0131926,-0.0657666,
			-0.0331817,-0.730369,-0.0657666,
			1.21331,-1.43358,-0.0657666,
			2.5118,-0.917119,-0.0916115,
			3.06333,-1.12383,0.450235,
			3.43266,-0.167953,0.450235,
			3.43266,0.0131926,0.450235,
			3.31158,0.0131926,2.00763,
			3.31158,0.222706,2.00763,
			3.18953,0.265519,2.00763,
			2.66054,1.27398,0.409614,
			1.21288,1.56324,0.566692,
			-0.0613496,0.873195,0.530222,
			-0.244426,0.0131926,0.530222,
			-0.0613496,-0.84681,0.530222,
			1.21288,-1.53686,0.566692,
			2.66054,-1.24759,0.409614,
			3.18953,-0.239134,2.00763,
			3.31158,-0.19632,2.00763,
			3.31158,0.0131926,2.00763,
			2.82922,0.0131926,1.43919,
			2.82922,0.0634254,1.43919,
			2.88211,0.124312,1.39263,
			1.12024,0.632184,0.332866,
			1.19494,1.26056,0.851303,
			0.0960279,0.60794,0.857131,
			-0.0446044,0.0131926,0.857131,
			0.0960279,-0.581555,0.857131,
			1.19494,-1.23418,0.851303,
			1.12024,-0.618992,0.332866,
			2.88211,-0.0979265,1.39263,
			2.82922,-0.0370402,1.43919,
			2.82922,0.0131926,1.43919,
			2.53537,0.0131927,1.10909,
			2.53537,0.194424,1.10909,
			2.42558,0.620334,1.04763,
			2.01157,0.801663,0.908186,
			1.15419,0.971482,0.98254,
			0.350506,0.470106,1.03237,
			0.245852,0.0131927,1.03238,
			0.350506,-0.44372,1.03238,
			1.15419,-0.945097,0.98254,
			2.01157,-0.775277,0.908186,
			2.42558,-0.593949,1.04763,
			2.53537,-0.168039,1.10909,
			2.53537,0.0131927,1.10909,
			1.78151,0.0131927,0.94063,
			1.78151,0.0883495,0.94063,
			1.77363,0.260454,0.948305,
			1.5189,0.69632,1.10191,
			1.12675,0.782575,1.08713,
			0.541466,0.380035,1.13032,
			0.457467,0.0131927,1.13032,
			0.541466,-0.353649,1.13032,
			1.12675,-0.75619,1.08713,
			1.5189,-0.669934,1.10191,
			1.77363,-0.234068,0.948305,
			1.78151,-0.0619642,0.94063,
			1.78151,0.0131927,0.94063,
			1.59703,0.0131926,1.26531,
			1.59703,0.0853004,1.26531,
			1.57395,0.248356,1.26856,
			1.37314,0.572456,1.20861,
			1.04665,0.609796,1.17838,
			0.619437,0.297653,1.27202,
			0.554301,0.0131927,1.27202,
			0.619437,-0.271268,1.27202,
			1.04665,-0.58341,1.17838,
			1.37314,-0.54607,1.20861,
			1.57395,-0.22197,1.26856,
			1.59703,-0.0589152,1.26531,
			1.59703,0.0131926,1.26531,
			1.51978,0.0131926,1.37211,
			1.51978,0.111102,1.37211,
			1.47815,0.331128,1.37378,
			1.37119,0.708727,1.34617,
			0.953251,0.729271,1.36483,
			0.53756,0.354619,1.37425,
			0.45938,0.0131926,1.37425,
			0.53756,-0.328234,1.37425,
			0.953251,-0.702886,1.36483,
			1.37119,-0.682342,1.34617,
			1.47815,-0.304743,1.37378,
			1.51978,-0.0847171,1.37211,
			1.51978,0.0131926,1.37211,
			1.90802,0.0131926,1.79656,
			1.90802,0.127865,1.79656,
			1.8532,0.384752,1.79691,
			1.3569,0.790178,1.82875,
			0.867368,0.796466,1.83264,
			0.249556,0.401946,1.83461,
			0.16404,0.0131926,1.83461,
			0.249556,-0.375561,1.83461,
			0.867368,-0.770081,1.83264,
			1.3569,-0.763793,1.82875,
			1.8532,-0.358367,1.79691,
			1.90802,-0.10148,1.79656,
			1.90802,0.0131926,1.79656,
			1.70755,0.0131926,2.42201,
			1.70755,0.111781,2.42201,
			1.65928,0.332482,2.42201,
			1.29612,0.674173,2.42201,
			0.868507,0.676264,2.42201,
			0.55633,0.430641,2.42201,
			0.293367,0.0131926,2.42201,
			0.55633,-0.404256,2.42201,
			0.868507,-0.649879,2.42201,
			1.29612,-0.647788,2.42201,
			1.65928,-0.306097,2.42201,
			1.70755,-0.0853954,2.42201,
			1.70755,0.0131926,2.42201,
			1.0955,0.0131926,2.51111,
			1.0955,0.0201538,2.51111,
			1.09209,0.0357371,2.51111,
			1.06727,0.0598634,2.51111,
			1.03513,0.060011,2.51111,
			1.01031,0.0355157,2.51111,
			1.00519,0.0131926,2.51111,
			1.01031,-0.00913046,2.51111,
			1.03513,-0.0336258,2.51111,
			1.06727,-0.0334782,2.51111,
			1.09209,-0.00935192,2.51111,
			1.0955,0.00623142,2.51111,
			1.0955,0.0131926,2.51111,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017
		};

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 13;
		this->vPoints	= 14;

		float tempuKnots[17]={-3.14159,-3.14159,-3.14159,-3.14159,-2.61799,-2.0944,-1.0472,
			-0.523599,6.66134e-016,0.523599,1.0472,2.0944,2.61799,3.14159,3.14159,3.14159,3.14159};

		float tempvKnots[18]={-1.5708,-1.5708,-1.5708,-1.5708,-1.0472,-0.523599,0,0.523599,0.808217,
			1.04015,1.0472,1.24824,1.29714,1.46148,1.5708,1.5708,1.5708,1.5708};

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];
		for (int v = 0; v <this->vPoints ; v++)
		{
			for (int u = 0; u < this->uPoints; u++)
			{
				ctlpoints[v*this->uPoints*4+u*4+0] = (tempctlpoints[v][u][0]-1.5)*100;
				ctlpoints[v*this->uPoints*4+u*4+1] = (tempctlpoints[v][u][1])*100;
				ctlpoints[v*this->uPoints*4+u*4+2] = (tempctlpoints[v][u][2]-0.5)*100;
				ctlpoints[v*this->uPoints*4+u*4+3] = weights[v][u];
			}
		}
		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[16]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[17]-tempvKnots[0]);

		//Trimming for the outline
		float tempCntlPoints[15] = {0,0,1,1,0,1,1,1,1,0,1,1,0,0,1};
		for(int k=0; k<4; k++)
		{
			BSpline* bSplineCurve		= new BSpline();
			bSplineCurve->nPoints		= 2;
			bSplineCurve->order			= 2;
			int numKnots				= 4;
			bSplineCurve->cntlPoints	= new float[6];
			bSplineCurve->knotVector	= new float[numKnots];
			float tempKnots[4]			= {0,0,1,1};
			bSplineCurve->cntlPoints[0] = tempCntlPoints[k*3+0];
			bSplineCurve->cntlPoints[1] = tempCntlPoints[k*3+1];
			bSplineCurve->cntlPoints[2] = tempCntlPoints[k*3+2];
			bSplineCurve->cntlPoints[3] = tempCntlPoints[k*3+3];
			bSplineCurve->cntlPoints[4] = tempCntlPoints[k*3+4];
			bSplineCurve->cntlPoints[5] = tempCntlPoints[k*3+5];
			for (int u = 0; u < 4; u++)
				bSplineCurve->knotVector[u] = tempKnots[u];
			this->trimCurves.push_back(bSplineCurve);
		}

		//Trimming for the eyes
		BSpline* bSplineCurve1		= new BSpline();
		bSplineCurve1->nPoints		= 9;
		bSplineCurve1->order		= 3;
		int numKnots1				= bSplineCurve1->nPoints + bSplineCurve1->order;
		bSplineCurve1->cntlPoints	= new float[bSplineCurve1->nPoints*3];
		bSplineCurve1->knotVector	= new float[numKnots1];
		float tempKnots1[12]		= {0,0,0,0.25,0.25,0.5,0.5,0.75,0.75,1,1,1};
		for (int u = 0; u < bSplineCurve1->nPoints; u++)
		{
			bSplineCurve1->cntlPoints[u*3+0] = .43 + cos(u*PI/4)*.02;
			bSplineCurve1->cntlPoints[u*3+1] = .93 + sin(u*PI/4)*.02;
			bSplineCurve1->cntlPoints[u*3+2] = (u%2 == 0? 1 : 1.5);
		}
		for (int u = 0; u < numKnots1; u++)
			bSplineCurve1->knotVector[u] = (tempKnots1[u]-tempKnots1[0])/(tempKnots1[numKnots1-1]-tempKnots1[0]);
		this->trimCurves.push_back(bSplineCurve1);

		BSpline* bSplineCurve2		= new BSpline();
		bSplineCurve2->nPoints		= 9;
		bSplineCurve2->order		= 3;
		int numKnots2				= bSplineCurve2->nPoints + bSplineCurve2->order;
		bSplineCurve2->cntlPoints	= new float[bSplineCurve2->nPoints*3];
		bSplineCurve2->knotVector	= new float[numKnots2];
		float tempKnots2[12]		= {0,0,0,0.5*PI,0.5*PI,PI,PI,1.5*PI,1.5*PI,2*PI,2*PI,2*PI};
		for (int u = 0; u < bSplineCurve2->nPoints; u++)
		{
			bSplineCurve2->cntlPoints[u*3+0] = .57+cos(u*PI/4)*.02;
			bSplineCurve2->cntlPoints[u*3+1] = .93+sin(u*PI/4)*.02;
			bSplineCurve2->cntlPoints[u*3+2] = (u%2 == 0? 1 : 1.5);
		}
		for (int u = 0; u < numKnots2; u++)
			bSplineCurve2->knotVector[u] = (tempKnots2[u]-tempKnots2[0])/(tempKnots2[numKnots2-1]-tempKnots2[0]);
		this->trimCurves.push_back(bSplineCurve2);
	}
	else if (surfNum==2)
	{
		this->trimmed=false;
		this->uBaseNum=5;
		this->vBaseNum=5;

		this->kdColor = Float3(0.768628, 0.462745, 0.137255);
		this->ksColor = Float3(0.9, 0.9, 0.9);
		this->ka = 0.11;
		this->shininess = 50;

		/*		Transform t;
		t.scale = 100;
		t.type = 3;
		this->transforms.push_back(t);
		t.translate  = Float3(-1.5,-1,0);
		t.type	= 1;
		this->transforms.push_back(t);
		t.angle = -90;
		t.axis=Float3(1,0,0);
		t.type = 2;
		this->transforms.push_back(t);
		*/
		float tempctlpoints[9][10][3]={
			-0.279652,0.207397,1.75598,-0.291276,0.195828,1.75911,-0.305767,0.156964,1.76536,-0.323195,0.0976051,1.77348,
			-0.324811,0.0575112,1.7749,-0.324878,0.00965657,1.77496,-0.323096,-0.0303954,1.7734,-0.305497,-0.0905963,1.76504,
			-0.291682,-0.12612,1.75909,-0.279652,-0.138572,1.75598,-0.269113,0.222966,1.74866,-0.263871,0.258906,1.77657,
			-0.3196,0.232048,1.81233,-0.398219,0.161683,1.84252,-0.413466,0.081587,1.84607,-0.41368,-0.0156633,1.84616,
			-0.398615,-0.0957014,1.84248,-0.319645,-0.166603,1.81134,-0.262126,-0.192803,1.77911,-0.269113,-0.154139,1.74866,
			-0.242427,0.248091,1.74002,-0.250205,0.313952,1.78585,-0.206038,0.298954,1.83128,-0.288104,0.183633,1.93874,
			-0.310232,0.0882907,1.94062,-0.310299,-0.023674,1.94074,-0.288436,-0.119573,1.93833,-0.206498,-0.233814,1.82973,
			-0.246528,-0.250884,1.7902,-0.242427,-0.179251,1.74002,-0.179498,0.282063,1.72166,-0.17679,0.326377,1.75992,
			-0.110471,0.312635,1.79867,-0.254015,0.182324,1.82418,-0.251274,0.0829962,1.82146,-0.251233,-0.0178177,1.82168,
			-0.255262,-0.119254,1.8228,-0.111415,-0.246282,1.797,-0.173192,-0.26317,1.76401,-0.179498,-0.21315,1.72166,
			-0.0461346,0.30226,1.6761,-0.0309577,0.346416,1.72013,-0.0164148,0.32483,1.77109,-0.0589178,0.175862,1.80591,
			-0.0531642,0.0769225,1.81173,-0.0531119,-0.0109558,1.8121,-0.0603414,-0.11356,1.80335,-0.017706,-0.257577,1.76904,
			-0.0288649,-0.282457,1.72533,-0.0461346,-0.233217,1.6761,0.223016,0.355831,1.68021,0.220378,0.396637,1.72855,
			0.177912,0.348447,1.78214,0.111397,0.163111,1.8289,0.114733,0.0725392,1.86208,0.114717,-0.00559361,1.86264,
			0.111044,-0.101223,1.82465,0.176651,-0.280222,1.77884,0.219996,-0.331394,1.73441,0.223016,-0.286637,1.68021,
			0.477456,0.492685,1.69775,0.478012,0.525066,1.7464,0.389698,0.419772,1.79903,0.241845,0.157044,1.86838,
			0.248091,0.082689,1.948,0.248091,-0.0151387,1.94871,0.243166,-0.0948644,1.86227,0.388089,-0.350312,1.79395,
			0.478027,-0.458802,1.7521,0.477456,-0.423169,1.69775,0.625729,0.586266,1.7479,0.624419,0.607445,1.79189,
			0.485322,0.455259,1.82632,0.257618,0.145814,1.89966,0.271645,0.105257,2.03203,0.271645,-0.038398,2.03279,
			0.261609,-0.082252,1.89132,0.484169,-0.385149,1.81866,0.624385,-0.541398,1.79651,0.625729,-0.517056,1.7479,
			0.657067,0.586266,1.7479,0.646467,0.575674,1.7919,0.502994,0.41679,1.82461,0.283455,0.142402,1.89403,
			0.303728,0.110488,2.02461,0.303728,-0.0442053,2.02538,0.286523,-0.077615,1.88597,0.502318,-0.349657,1.81709,
			0.646195,-0.50881,1.79653,0.657067,-0.517056,1.7479};

			this->uOrder	= 4;
			this->vOrder	= 4;
			this->uPoints	= 10;
			this->vPoints	= 9;

			float tempuKnots[14]={0,0,0,0,0.179541,0.317924,0.485586,0.507528,0.709398,0.813231,1,1,1,1};
			float tempvKnots[13]={0,0,0,0,0.145456,0.265731,0.436096,0.583258,0.847704,1,1,1,1};

			ctlpoints = new float[this->uPoints*this->vPoints*4];
			uKnots = new float[this->uPoints+this->uOrder];
			vKnots = new float[this->vPoints+this->vOrder];
			int u, v;
			for (v = 0; v <this->vPoints ; v++)
			{
				for (u = 0; u < this->uPoints; u++)
				{
					ctlpoints[v*this->uPoints*4+u*4+0] = (tempctlpoints[v][u][0]-1.5)*100;
					ctlpoints[v*this->uPoints*4+u*4+1] = (tempctlpoints[v][u][1])*100;
					ctlpoints[v*this->uPoints*4+u*4+2] = (tempctlpoints[v][u][2]-0.5)*100;
					ctlpoints[v*this->uPoints*4+u*4+3] = 1.0;
				}
			}
			for (u = 0; u < this->uPoints + this->uOrder; u++)
				uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
			for (v = 0; v < this->vPoints + this->vOrder; v++)
				vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);

	}
	else if (surfNum==3)
	{
		this->trimmed=false;
		this->uBaseNum=5;
		this->vBaseNum=5;

		this->kdColor = Float3(0.768628, 0.462745, 0.137255);
		this->ksColor = Float3(0.9, 0.9, 0.9);
		this->ka = 0.11;
		this->shininess = 50;

		float tempctlpoints[6][6][3]={
			0.697749,-0.482282,1.72233,0.697749,-0.551961,1.72629,0.497736,-0.357714,1.40084,0.497736,0.429409,1.40084,
			0.697749,0.623656,1.72629,0.697749,0.553978,1.72233,0.275414,-0.310712,1.69077,0.275414,-0.350082,1.66688,
			0.25645,-0.222548,1.48535,0.25645,0.294244,1.48535,0.275414,0.421778,1.66688,0.275414,0.382408,1.69077,
			0.146088,-0.237558,1.70237,0.120839,-0.266714,1.66702,0.0189557,-0.269827,1.5816,0.0189557,0.341523,1.5816,
			0.120839,0.33841,1.66702,0.146088,0.309254,1.70237,-0.0870671,-0.223076,1.68462,-0.117868,-0.254292,1.66378,
			-0.315615,-0.227541,1.69583,-0.315615,0.299237,1.69583,-0.117868,0.325988,1.66378,-0.0870671,0.294772,1.68462,
			-0.231993,-0.185237,1.74138,-0.280097,-0.23448,1.73119,-0.380927,-0.0267278,1.75395,-0.380927,0.0984238,1.75395,
			-0.280097,0.306176,1.73119,-0.231993,0.256933,1.74138,-0.274698,-0.102977,1.76242,-0.299816,-0.0724353,1.77409,
			-0.307722,-0.000246432,1.77549,-0.307722,0.0719424,1.77549,-0.299816,0.144131,1.77409,-0.274698,0.174673,1.76242};

			this->uOrder	= 4;
			this->vOrder	= 4;
			this->uPoints	= 6;
			this->vPoints	= 6;

			float tempuKnots[10]={0,0,0,0,0.333333,0.666667,1,1,1,1};
			float tempvKnots[10]={0,0,0,0,0.333333,0.666667,1,1,1,1};

			ctlpoints = new float[this->uPoints*this->vPoints*4];
			uKnots = new float[this->uPoints+this->uOrder];
			vKnots = new float[this->vPoints+this->vOrder];
			int u, v;
			for (v = 0; v <this->vPoints ; v++)
			{
				for (u = 0; u < this->uPoints; u++)
				{
					ctlpoints[v*this->uPoints*4+u*4+0] = (tempctlpoints[v][u][0]-1.5)*100;
					ctlpoints[v*this->uPoints*4+u*4+1] = (tempctlpoints[v][u][1])*100;
					ctlpoints[v*this->uPoints*4+u*4+2] = (tempctlpoints[v][u][2]-0.5)*100;
					ctlpoints[v*this->uPoints*4+u*4+3] = 1.0;
				}
			}
			for (u = 0; u < this->uPoints + this->uOrder; u++)
				uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
			for (v = 0; v < this->vPoints + this->vOrder; v++)
				vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==4)
	{
		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.64, 0.48, 0.24);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;


		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 6;

		float tempvKnots[10]={0,0,0,0,1,2,3,3,3,3};
		float tempuKnots[10]={0,0,0,0,1,2,3,3,3,3};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] =  -50; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] =   50; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] =  -50; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] =   50; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] =  -50; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=3; ctlpoints[v*uPoints*4+u*4+0] =   50; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=3; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=2; ctlpoints[v*uPoints*4+u*4+0] =  -50; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=3; ctlpoints[v*uPoints*4+u*4+0] =   50; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=4; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=2; ctlpoints[v*uPoints*4+u*4+0] =  -50; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=3; ctlpoints[v*uPoints*4+u*4+0] =   50; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=5; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=2; ctlpoints[v*uPoints*4+u*4+0] =  -50; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=3; ctlpoints[v*uPoints*4+u*4+0] =   50; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==5)
	{
		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.80, 0.24, 0.24);
		this->ksColor = Float3(0.12, 0.06, 0.06);
		this->ka = 0.5;
		this->shininess = 1000;

		//		this->uOrder	= 2;
		//		this->vOrder	= 2;
		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 6;

		//		float tempvKnots[8]={0,0,1,2,3,4,5,5};
		//		float tempuKnots[8]={0,0,1,2,3,4,5,5};
		float tempvKnots[10]={0,0,0,0,1,2,3,3,3,3};
		float tempuKnots[10]={0,0,0,0,1,2,3,3,3,3};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=0; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=0; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=0; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=1; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=1; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=1; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=2; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=2; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=2; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=3; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=3; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=3; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=3; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=4; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=4; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=4; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=5; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=5; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=5; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==6)
	{
		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.48, 0.24, 0.64);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;

		//		this->uOrder	= 2;
		//		this->vOrder	= 2;
		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 6;

		//		float tempvKnots[8]={0,0,1,2,3,4,5,5};
		//		float tempuKnots[8]={0,0,1,2,3,4,5,5};
		float tempvKnots[10]={0,0,0,0,1,2,3,3,3,3};
		float tempuKnots[10]={0,0,0,0,1,2,3,3,3,3};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=0; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=0; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=0; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -200; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=1; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -200; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=1; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=1; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -200; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -100; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -100; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -100; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=2; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -100; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=2; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -100; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=2; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -100; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=3; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=3; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=3; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=3; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = -100; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=4; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=4; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=4; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 200; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=5; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=5; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=5; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==7)
	{
		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.48, 0.24, 0.64);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 6;

		float tempvKnots[10]={0,0,0,0,1,2,3,3,3,3};
		float tempuKnots[10]={0,0,0,0,1,2,3,3,3,3};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=0; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=0; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=0; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=1; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=1; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=1; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=2; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=2; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=2; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=3; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=3; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=3; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=3; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=4; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=4; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=4; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=5; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=5; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=5; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==8)
	{
		this->trimmed = false;
		this->uBaseNum = 20;
		this->vBaseNum = 20;

		this->kdColor = Float3(0.48, 0.24, 0.64);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 3;
		this->vOrder	= 3;
		this->uPoints	= 3;
		this->vPoints	= 3;

		float tempvKnots[10]={0,0,0,1,1,1};
		float tempuKnots[10]={0,0,0,1,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1.0/sqrt(2.0);
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1.0/sqrt(2.0);
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1.0/sqrt(2.0);
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==9)
	{
		this->trimmed = false;
		this->uBaseNum = 20;
		this->vBaseNum = 20;

		this->kdColor = Float3(0.48, 0.24, 0.64);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 2;
		this->vOrder	= 3;
		this->uPoints	= 2;
		this->vPoints	= 3;

		float tempuKnots[10]={0,0,1,1};
		float tempvKnots[10]={0,0,0,1,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1.0/sqrt(2.0);
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0   ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 0   ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1.0/sqrt(2.0);
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = 100 ; ctlpoints[v*uPoints*4+u*4+1] = 100 ; ctlpoints[v*uPoints*4+u*4+2] = 0  ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);

	}
	else if (surfNum==10)
	{
		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.64, 0.48, 0.24);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 6;

		float tempvKnots[10]={0,0,0,0,1,2,3,3,3,3};
		float tempuKnots[10]={0,0,0,0,1,2,3,3,3,3};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=0; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=0; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=0; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=1; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=1; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=1; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=2; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=2; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=2; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=3; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=3; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=3; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=3; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=4; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=4; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=4; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=5; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=5; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=5; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==11)
	{
		this->trimmed = true;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.48, 0.24, 0.64);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 6;

		float tempvKnots[10]={0,0,0,0,1,2,3,3,3,3};
		float tempuKnots[10]={0,0,0,0,1,2,3,3,3,3};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;

		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=0; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=0; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=0; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=1; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=1; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=1; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -150; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=2; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=2; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=2; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = -50; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=3; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=3; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=3; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=3; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 50; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 350 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=4; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=4; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=4; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=4; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 150; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] = -150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=5; ctlpoints[v*uPoints*4+u*4+0] = -50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=3; u=5; ctlpoints[v*uPoints*4+u*4+0] =  50 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 400 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=4; u=5; ctlpoints[v*uPoints*4+u*4+0] =  150; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300 ; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=5; u=5; ctlpoints[v*uPoints*4+u*4+0] =  250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 450 ; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);


		//Trimming for the outline
		float tempCntlPoints[15] = {0,0,1,1,0,1,1,1,1,0,1,1,0,0,1};
		for(int k=0; k<4; k++)
		{
			BSpline* bSplineCurve		= new BSpline();
			bSplineCurve->nPoints		= 2;
			bSplineCurve->order			= 2;
			int numKnots				= 4;
			bSplineCurve->cntlPoints	= new float[6];
			bSplineCurve->knotVector	= new float[numKnots];
			float tempKnots[4]			= {0,0,1,1};
			bSplineCurve->cntlPoints[0] = tempCntlPoints[k*3+0];
			bSplineCurve->cntlPoints[1] = tempCntlPoints[k*3+1];
			bSplineCurve->cntlPoints[2] = tempCntlPoints[k*3+2];
			bSplineCurve->cntlPoints[3] = tempCntlPoints[k*3+3];
			bSplineCurve->cntlPoints[4] = tempCntlPoints[k*3+4];
			bSplineCurve->cntlPoints[5] = tempCntlPoints[k*3+5];
			for (int u = 0; u < 4; u++)
				bSplineCurve->knotVector[u] = tempKnots[u];
			this->trimCurves.push_back(bSplineCurve);
		}

		//Trimming for the trim
		BSpline* bSplineCurve1		= new BSpline();
		bSplineCurve1->nPoints		= 9;
		bSplineCurve1->order		= 3;
		int numKnots1				= bSplineCurve1->nPoints + bSplineCurve1->order;
		bSplineCurve1->cntlPoints	= new float[bSplineCurve1->nPoints*3];
		bSplineCurve1->knotVector	= new float[numKnots1];
		float tempKnots1[12]		= {0,0,0,0.25,0.25,0.5,0.5,0.75,0.75,1,1,1};
		for (int u = 0; u < bSplineCurve1->nPoints; u++)
		{
			bSplineCurve1->cntlPoints[u*3+0] = 0.9 + cos(u*PI/4)*.2;
			bSplineCurve1->cntlPoints[u*3+1] = 0.5 + sin(u*PI/4)*.2;
			bSplineCurve1->cntlPoints[u*3+2] = (u%2 == 0? 1 : 1.5);
		}
		for (int u = 0; u < numKnots1; u++)
			bSplineCurve1->knotVector[u] = (tempKnots1[u]-tempKnots1[0])/(tempKnots1[numKnots1-1]-tempKnots1[0]);
		this->trimCurves.push_back(bSplineCurve1);

		BSpline* bSplineCurve2		= new BSpline();
		bSplineCurve2->nPoints		= 9;
		bSplineCurve2->order		= 3;
		int numKnots2				= bSplineCurve2->nPoints + bSplineCurve2->order;
		bSplineCurve2->cntlPoints	= new float[bSplineCurve2->nPoints*3];
		bSplineCurve2->knotVector	= new float[numKnots2];
		float tempKnots2[12]		= {0,0,0,0.5*PI,0.5*PI,PI,PI,1.5*PI,1.5*PI,2*PI,2*PI,2*PI};
		for (int u = 0; u < bSplineCurve2->nPoints; u++)
		{
			bSplineCurve2->cntlPoints[u*3+0] = 0 + cos(u*PI/4)*.2;
			bSplineCurve2->cntlPoints[u*3+1] = 0 + sin(u*PI/4)*.2;
			bSplineCurve2->cntlPoints[u*3+2] = (u%2 == 0? 1 : 1.5);
		}
		for (int u = 0; u < numKnots2; u++)
			bSplineCurve2->knotVector[u] = (tempKnots2[u]-tempKnots2[0])/(tempKnots2[numKnots2-1]-tempKnots2[0]);
		this->trimCurves.push_back(bSplineCurve2);


	}
	else if (surfNum==12)
	{
		this->trimmed=false;
		this->uBaseNum=15;
		this->vBaseNum=30;

		this->kdColor = Float3(1.0, 0.5, 0.0);
		this->ksColor = Float3(0.9, 0.5, 0.1);
		this->ka = 0.1;
		this->shininess = 50;

		float weights[14][13]={1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.5,1,1,1,1,1,0.5,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

		float tempctlpoints[14][13][3]={
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.49662,0.0131926,-0.535702,
			1.92379,0.0131926,-0.535702,
			1.92379,0.0465361,-0.535702,
			1.90747,0.121179,-0.535702,
			1.57358,0.236742,-0.535702,
			1.41966,0.237449,-0.535702,
			1.30075,0.120118,-0.535702,
			1.27627,0.0131926,-0.535702,
			1.30075,-0.093733,-0.535702,
			1.41966,-0.211064,-0.535702,
			1.57358,-0.210357,-0.535702,
			1.90747,-0.0947938,-0.535702,
			1.92379,-0.0201509,-0.535702,
			1.92379,0.0131926,-0.535702,
			2.73528,0.0131926,-0.516333,
			2.73528,0.118624,-0.516333,
			2.6288,0.354652,-0.516333,
			1.80454,0.720067,-0.516333,
			1.00825,1.13359,-0.516333,
			0.455347,0.351297,-0.516333,
			0.295617,0.0131926,-0.516333,
			0.455347,-0.324912,-0.516333,
			1.00825,-1.1072,-0.516333,
			1.80454,-0.693681,-0.516333,
			2.6288,-0.328267,-0.516333,
			2.73528,-0.092239,-0.516333,
			2.73528,0.0131926,-0.516333,
			3.43266,0.0131926,0.450235,
			3.43266,0.194338,0.450235,
			3.06333,1.15022,0.450235,
			2.5118,0.943504,-0.0916114,
			1.21331,1.66,-0.0657666,
			-0.0331817,0.9,-0.0657666,
			-0.225645,0.0131926,-0.0657666,
			-0.0331817,-0.9,-0.0657666,
			1.21331,-1.66,-0.0657666,
			2.5118,-0.917119,-0.0916115,
			3.06333,-1.12383,0.450235,
			3.43266,-0.167953,0.450235,
			3.43266,0.0131926,0.450235,
			3.31158,0.0131926,1.5,
			3.31158,0.222706,1.5,
			3.18953,0.265519,1.5,
			2.66054,1.27398,0.409614,
			1.21288,1.86324,0.566692,
			-0.0613496,1.073195,0.530222,
			-0.244426,0.0131926,0.530222,
			-0.0613496,-1.04681,0.530222,
			1.21288,-1.83686,0.566692,
			2.66054,-1.24759,0.409614,
			3.18953,-0.239134,1.5,
			3.31158,-0.19632,1.5,
			3.31158,0.0131926,1.5,
			2.82922,0.0131926,1.43919,
			2.82922,0.0634254,1.43919,
			2.88211,0.124312,1.39263,
			1.12024,0.632184,0.332866,
			1.19494,1.26056,0.851303,
			0.0960279,0.60794,0.857131,
			-0.0446044,0.0131926,0.857131,
			0.0960279,-0.581555,0.857131,
			1.19494,-1.23418,0.851303,
			1.12024,-0.618992,0.332866,
			2.88211,-0.0979265,1.39263,
			2.82922,-0.0370402,1.43919,
			2.82922,0.0131926,1.43919,
			2.53537,0.0131927,1.10909,
			2.53537,0.194424,1.10909,
			2.42558,0.620334,1.04763,
			2.01157,0.801663,0.908186,
			1.15419,0.971482,0.98254,
			0.350506,0.470106,1.03237,
			0.245852,0.0131927,1.03238,
			0.350506,-0.44372,1.03238,
			1.15419,-0.945097,0.98254,
			2.01157,-0.775277,0.908186,
			2.42558,-0.593949,1.04763,
			2.53537,-0.168039,1.10909,
			2.53537,0.0131927,1.10909,
			1.78151,0.0131927,0.94063,
			1.78151,0.0883495,0.94063,
			1.77363,0.260454,0.948305,
			1.5189,0.69632,1.10191,
			1.12675,0.782575,1.08713,
			0.541466,0.380035,1.13032,
			0.457467,0.0131927,1.13032,
			0.541466,-0.353649,1.13032,
			1.12675,-0.75619,1.08713,
			1.5189,-0.669934,1.10191,
			1.77363,-0.234068,0.948305,
			1.78151,-0.0619642,0.94063,
			1.78151,0.0131927,0.94063,
			1.59703,0.0131926,1.26531,
			1.59703,0.0853004,1.26531,
			1.57395,0.248356,1.26856,
			1.37314,0.572456,1.20861,
			1.04665,0.609796,1.17838,
			0.619437,0.297653,1.27202,
			0.554301,0.0131927,1.27202,
			0.619437,-0.271268,1.27202,
			1.04665,-0.58341,1.17838,
			1.37314,-0.54607,1.20861,
			1.57395,-0.22197,1.26856,
			1.59703,-0.0589152,1.26531,
			1.59703,0.0131926,1.26531,
			1.51978,0.0131926,1.37211,
			1.51978,0.111102,1.37211,
			1.47815,0.331128,1.37378,
			1.37119,0.708727,1.34617,
			0.953251,0.729271,1.36483,
			0.53756,0.354619,1.37425,
			0.45938,0.0131926,1.37425,
			0.53756,-0.328234,1.37425,
			0.953251,-0.702886,1.36483,
			1.37119,-0.682342,1.34617,
			1.47815,-0.304743,1.37378,
			1.51978,-0.0847171,1.37211,
			1.51978,0.0131926,1.37211,
			1.90802,0.0131926,1.79656,
			1.90802,0.127865,1.79656,
			1.8532,0.384752,1.79691,
			1.3569,0.790178,1.82875,
			0.867368,0.796466,1.83264,
			0.249556,0.401946,1.83461,
			0.16404,0.0131926,1.83461,
			0.249556,-0.375561,1.83461,
			0.867368,-0.770081,1.83264,
			1.3569,-0.763793,1.82875,
			1.8532,-0.358367,1.79691,
			1.90802,-0.10148,1.79656,
			1.90802,0.0131926,1.79656,
			1.70755,0.0131926,2.42201,
			1.70755,0.111781,2.42201,
			1.65928,0.332482,2.42201,
			1.29612,0.674173,2.42201,
			0.868507,0.676264,2.42201,
			0.55633,0.430641,2.42201,
			0.293367,0.0131926,2.42201,
			0.55633,-0.404256,2.42201,
			0.868507,-0.649879,2.42201,
			1.29612,-0.647788,2.42201,
			1.65928,-0.306097,2.42201,
			1.70755,-0.0853954,2.42201,
			1.70755,0.0131926,2.42201,
			1.0955,0.0131926,2.51111,
			1.0955,0.0201538,2.51111,
			1.09209,0.0357371,2.51111,
			1.06727,0.0598634,2.51111,
			1.03513,0.060011,2.51111,
			1.01031,0.0355157,2.51111,
			1.00519,0.0131926,2.51111,
			1.01031,-0.00913046,2.51111,
			1.03513,-0.0336258,2.51111,
			1.06727,-0.0334782,2.51111,
			1.09209,-0.00935192,2.51111,
			1.0955,0.00623142,2.51111,
			1.0955,0.0131926,2.51111,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017,
			1.0512,0.0131926,2.5017
		};

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 13;
		this->vPoints	= 14;

		float tempuKnots[17]={-3.14159,-3.14159,-3.14159,-3.14159,-2.61799,-2.0944,-1.0472,
			-0.523599,6.66134e-016,0.523599,1.0472,2.0944,2.61799,3.14159,3.14159,3.14159,3.14159};

		float tempvKnots[18]={-1.5708,-1.5708,-1.5708,-1.5708,-1.0472,-0.523599,0,0.523599,0.808217,
			1.04015,1.0472,1.24824,1.29714,1.46148,1.5708,1.5708,1.5708,1.5708};

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];
		int u, v;
		for (v = 0; v <this->vPoints ; v++)
		{
			for (u = 0; u < this->uPoints; u++)
			{
				ctlpoints[v*this->uPoints*4+u*4+0] = (tempctlpoints[v][u][0]-1.5)*100;
				ctlpoints[v*this->uPoints*4+u*4+1] = (tempctlpoints[v][u][1])*100;
				ctlpoints[v*this->uPoints*4+u*4+2] = (tempctlpoints[v][u][2]-0.5)*100;
				ctlpoints[v*this->uPoints*4+u*4+3] = weights[v][u];
			}
		}
		for (u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[16]-tempuKnots[0]);
		for (v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[17]-tempvKnots[0]);

		//Trimming for the outline
		float tempCntlPoints[15] = {0,0,1,1,0,1,1,1,1,0,1,1,0,0,1};
		for(int k=0; k<4; k++)
		{
			BSpline* bSplineCurve		= new BSpline();
			bSplineCurve->nPoints		= 2;
			bSplineCurve->order			= 2;
			int numKnots				= 4;
			bSplineCurve->cntlPoints	= new float[6];
			bSplineCurve->knotVector	= new float[numKnots];
			float tempKnots[4]			= {0,0,1,1};
			bSplineCurve->cntlPoints[0] = tempCntlPoints[k*3+0];
			bSplineCurve->cntlPoints[1] = tempCntlPoints[k*3+1];
			bSplineCurve->cntlPoints[2] = tempCntlPoints[k*3+2];
			bSplineCurve->cntlPoints[3] = tempCntlPoints[k*3+3];
			bSplineCurve->cntlPoints[4] = tempCntlPoints[k*3+4];
			bSplineCurve->cntlPoints[5] = tempCntlPoints[k*3+5];
			for (int u = 0; u < 4; u++)
				bSplineCurve->knotVector[u] = tempKnots[u];
			this->trimCurves.push_back(bSplineCurve);
		}

		//Trimming for the eyes
		BSpline* bSplineCurve1		= new BSpline();
		bSplineCurve1->nPoints		= 9;
		bSplineCurve1->order		= 3;
		int numKnots1				= bSplineCurve1->nPoints + bSplineCurve1->order;
		bSplineCurve1->cntlPoints	= new float[bSplineCurve1->nPoints*3];
		bSplineCurve1->knotVector	= new float[numKnots1];
		float tempKnots1[12]		= {0,0,0,0.25,0.25,0.5,0.5,0.75,0.75,1,1,1};
		for (int u = 0; u < bSplineCurve1->nPoints; u++)
		{
			bSplineCurve1->cntlPoints[u*3+0] = .43 + cos(u*PI/4)*.02;
			bSplineCurve1->cntlPoints[u*3+1] = .93 + sin(u*PI/4)*.02;
			bSplineCurve1->cntlPoints[u*3+2] = (u%2 == 0? 1 : 1.5);
		}
		for (int u = 0; u < numKnots1; u++)
			bSplineCurve1->knotVector[u] = (tempKnots1[u]-tempKnots1[0])/(tempKnots1[numKnots1-1]-tempKnots1[0]);
		this->trimCurves.push_back(bSplineCurve1);

		BSpline* bSplineCurve2		= new BSpline();
		bSplineCurve2->nPoints		= 9;
		bSplineCurve2->order		= 3;
		int numKnots2				= bSplineCurve2->nPoints + bSplineCurve2->order;
		bSplineCurve2->cntlPoints	= new float[bSplineCurve2->nPoints*3];
		bSplineCurve2->knotVector	= new float[numKnots2];
		float tempKnots2[12]		= {0,0,0,0.5*PI,0.5*PI,PI,PI,1.5*PI,1.5*PI,2*PI,2*PI,2*PI};
		for (int u = 0; u < bSplineCurve2->nPoints; u++)
		{
			bSplineCurve2->cntlPoints[u*3+0] = .57+cos(u*PI/4)*.02;
			bSplineCurve2->cntlPoints[u*3+1] = .93+sin(u*PI/4)*.02;
			bSplineCurve2->cntlPoints[u*3+2] = (u%2 == 0? 1 : 1.5);
		}
		for (int u = 0; u < numKnots2; u++)
			bSplineCurve2->knotVector[u] = (tempKnots2[u]-tempKnots2[0])/(tempKnots2[numKnots2-1]-tempKnots2[0]);
		this->trimCurves.push_back(bSplineCurve2);
	}
	///////////////////////
	//iddo: additional Bezier-type surfaces (no inner knots) for testing
	else if (surfNum==13)
	{
		this->trimmed = false;
		this->uBaseNum=8;
		this->vBaseNum=8;

		this->kdColor = Float3(0.80, 0.24, 0.24);
		this->ksColor = Float3(0.12, 0.06, 0.06);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 3;
		this->vOrder	= 3;
		this->uPoints	= 3;
		this->vPoints	= 3;

		float tempvKnots[10]={0,0,0,1,1,1};
		float tempuKnots[10]={0,0,0,1,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 100; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==14)
	{
		this->trimmed = false;
		this->uBaseNum=8;
		this->vBaseNum=8;

		this->kdColor = Float3(0.80, 0.24, 0.24);
		this->ksColor = Float3(0.12, 0.06, 0.06);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 3;
		this->vOrder	= 3;
		this->uPoints	= 3;
		this->vPoints	= 3;

		float tempvKnots[10]={0,0,0,1,1,1};
		float tempuKnots[10]={0,0,0,1,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==15)
	{
		this->trimmed = false;
		this->uBaseNum=8;
		this->vBaseNum=8;

		this->kdColor = Float3(0.80, 0.24, 0.24);
		this->ksColor = Float3(0.12, 0.06, 0.06);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 3;
		this->vOrder	= 3;
		this->uPoints	= 3;
		this->vPoints	= 3;

		float tempvKnots[10]={0,0,0,1,1,1};
		float tempuKnots[10]={0,0,0,1,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=0; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = -250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=1; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = -250; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=2; u=2; ctlpoints[v*uPoints*4+u*4+0] = 250 ; ctlpoints[v*uPoints*4+u*4+1] = 250; ctlpoints[v*uPoints*4+u*4+2] = 200; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	////////////////////////
	//iddo: Hausdorff example for attaining a HD close to KA
	else if (surfNum==17)
	{
		//256x256 plane at the origin, translated by 128.0/res (so the origin won't be a sample point)
		this->trimmed = false;
		this->uBaseNum=8;
		this->vBaseNum=8;

		this->kdColor = Float3( 1.0, 1.0, 0.0);
		this->ksColor = Float3( 1.0, 1.0, 0.0);
		this->ka = 1.0;
		this->shininess = 1000;

		this->uOrder	= 2;
		this->vOrder	= 2;
		this->uPoints	= 2;
		this->vPoints	= 2;

		float tempvKnots[4]={0,0,1,1};
		float tempuKnots[4]={0,0,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int res = 32;
		//int res = 16;
		double d = 128.0/res; //we want to translate the plane by (128/res,128/res) so the origin will be exactly between sample points.. 

		int u,v;
		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = -128+d; ctlpoints[v*uPoints*4+u*4+1] = -128+d; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = 128+d; ctlpoints[v*uPoints*4+u*4+1] = -128+d; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = -128+d; ctlpoints[v*uPoints*4+u*4+1] = 128+d; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 128+d; ctlpoints[v*uPoints*4+u*4+1] = 128+d; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==18)
	{
		//Cylinder (320 radius) around the z-axis:

		this->trimmed = false;
		this->uBaseNum=8;
		this->vBaseNum=8;

		this->kdColor = Float3( 1.0, 1.0, 1.0);
		this->ksColor = Float3( 1.0, 1.0, 1.0);
		this->ka = 1.0;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 2;
		this->uPoints	= 13;
		this->vPoints	= 2;

		float tempvKnots[4]={0, 0, 1, 1};
		float tempuKnots[17]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		v=0; u=0; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=0; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=1; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = 180; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=1; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = 180; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=2; ctlpoints[v*uPoints*4+u*4+0] = 180; ctlpoints[v*uPoints*4+u*4+1] = 320; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=2; ctlpoints[v*uPoints*4+u*4+0] = 180; ctlpoints[v*uPoints*4+u*4+1] = 320; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=3; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 320; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=3; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 320; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=4; ctlpoints[v*uPoints*4+u*4+0] = -180; ctlpoints[v*uPoints*4+u*4+1] = 320; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=4; ctlpoints[v*uPoints*4+u*4+0] = -180; ctlpoints[v*uPoints*4+u*4+1] = 320; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=5; ctlpoints[v*uPoints*4+u*4+0] = -320; ctlpoints[v*uPoints*4+u*4+1] = 180; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=5; ctlpoints[v*uPoints*4+u*4+0] = -320; ctlpoints[v*uPoints*4+u*4+1] = 180; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=6; ctlpoints[v*uPoints*4+u*4+0] = -320; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=6; ctlpoints[v*uPoints*4+u*4+0] = -320; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=7; ctlpoints[v*uPoints*4+u*4+0] = -320; ctlpoints[v*uPoints*4+u*4+1] = -180; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=7; ctlpoints[v*uPoints*4+u*4+0] = -320; ctlpoints[v*uPoints*4+u*4+1] = -180; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=8; ctlpoints[v*uPoints*4+u*4+0] = -180; ctlpoints[v*uPoints*4+u*4+1] = -320; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=8; ctlpoints[v*uPoints*4+u*4+0] = -180; ctlpoints[v*uPoints*4+u*4+1] = -320; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=9; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = -320; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=9; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = -320; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=10; ctlpoints[v*uPoints*4+u*4+0] = 180; ctlpoints[v*uPoints*4+u*4+1] = -320; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=10; ctlpoints[v*uPoints*4+u*4+0] = 180; ctlpoints[v*uPoints*4+u*4+1] = -320; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=11; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = -180; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=11; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = -180; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;

		v=0; u=12; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		v=1; u=12; ctlpoints[v*uPoints*4+u*4+0] = 320; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -300; ctlpoints[v*uPoints*4+u*4+3] = 1;


		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	///////////////////////
	//Utah teapot
	else if (surfNum==21)
	{
		// Body:
		// Scale of the Teapot
		float cntlScale = 100;

		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=20;

		this->kdColor = Float3(0.25, 0.24, 0.80);
		this->ksColor = Float3(0.06, 0.06, 0.12);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 10;
		this->vPoints	= 13;

		float tempuKnots[14]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3};
		float tempvKnots[17]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		u=0; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.784; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0.749; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0.805; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 0.98; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.784; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.749; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 1.3375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.805 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 1.4375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.98 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 1.75; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=2; ctlpoints[v*uPoints*4+u*4+0] = 1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=2; ctlpoints[v*uPoints*4+u*4+0] = 1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=2; ctlpoints[v*uPoints*4+u*4+0] = 1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 1.3375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 1.4375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 1.75; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.784; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.749; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 1.3375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.805 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 1.4375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.98 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 1.75; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=4; ctlpoints[v*uPoints*4+u*4+0] = -1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=4; ctlpoints[v*uPoints*4+u*4+0] = -1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=4; ctlpoints[v*uPoints*4+u*4+0] = -1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.784; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0.749; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0.805; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 0.98; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.784; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -0.749; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -0.805; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = -0.98; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=7; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = -1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=7; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = -1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=7; ctlpoints[v*uPoints*4+u*4+0] = -2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = -1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = -0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.784; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.749; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -1.3375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.805 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -1.4375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.98 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = -1.75; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=8; ctlpoints[v*uPoints*4+u*4+0] = -1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=8; ctlpoints[v*uPoints*4+u*4+0] = -1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=8; ctlpoints[v*uPoints*4+u*4+0] = -1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -1.3375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -1.4375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = -1.75; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.784; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.749; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -1.3375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.805 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -1.4375; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.98 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = -1.75; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=10; ctlpoints[v*uPoints*4+u*4+0] = 1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=10; ctlpoints[v*uPoints*4+u*4+0] = 1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=10; ctlpoints[v*uPoints*4+u*4+0] = 1.12 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = -2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.84 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -1.5; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.784; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -0.749; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = -0.805; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = -0.98; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=11; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = -1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=11; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = -1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=11; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = -1.12; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = -0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = -0.84; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.4; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.3375; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.4375 ; ctlpoints[v*uPoints*4+u*4+1] = 2.38125; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.75 ; ctlpoints[v*uPoints*4+u*4+1] = 1.725; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=12; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=12; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=7; v=12; ctlpoints[v*uPoints*4+u*4+0] = 2 ; ctlpoints[v*uPoints*4+u*4+1] = 0.3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=8; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0.075; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=9; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.5 ; ctlpoints[v*uPoints*4+u*4+1] = 0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);

		for (int v = 0; v <this->vPoints ; v++)
		{
			for (int u = 0; u < this->uPoints; u++)
			{
				ctlpoints[v*this->uPoints*4+u*4+0] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+1] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+2] *= cntlScale;
			}
		}


	}
	else if (surfNum==22)
	{
		// Spout:
		// Scale of the Teapot
		float cntlScale = 100;

		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.25, 0.24, 0.80);
		this->ksColor = Float3(0.06, 0.06, 0.12);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 7;
		this->vPoints	= 7;

		float tempuKnots[11]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2};
		float tempvKnots[11]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		u=0; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2.6; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2.3 ; ctlpoints[v*uPoints*4+u*4+1] = 1.95; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2.7 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2.9 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=0; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = 0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2.6; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = 0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2.3 ; ctlpoints[v*uPoints*4+u*4+1] = 1.95; ctlpoints[v*uPoints*4+u*4+2] = 0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2.7 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = 0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2.9 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = 0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=1; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=2; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 0.45; ctlpoints[v*uPoints*4+u*4+2] = 0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=2; ctlpoints[v*uPoints*4+u*4+0] = 3.1; ctlpoints[v*uPoints*4+u*4+1] = 0.675; ctlpoints[v*uPoints*4+u*4+2] = 0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=2; ctlpoints[v*uPoints*4+u*4+0] = 2.4 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=2; ctlpoints[v*uPoints*4+u*4+0] = 3.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=2; ctlpoints[v*uPoints*4+u*4+0] = 3.525 ; ctlpoints[v*uPoints*4+u*4+1] = 2.34375; ctlpoints[v*uPoints*4+u*4+2] = 0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=2; ctlpoints[v*uPoints*4+u*4+0] = 3.45 ; ctlpoints[v*uPoints*4+u*4+1] = 2.3625; ctlpoints[v*uPoints*4+u*4+2] = 0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=2; ctlpoints[v*uPoints*4+u*4+0] = 3.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=3; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 0.45; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=3; ctlpoints[v*uPoints*4+u*4+0] = 3.1; ctlpoints[v*uPoints*4+u*4+1] = 0.675; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=3; ctlpoints[v*uPoints*4+u*4+0] = 2.4 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=3; ctlpoints[v*uPoints*4+u*4+0] = 3.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=3; ctlpoints[v*uPoints*4+u*4+0] = 3.525 ; ctlpoints[v*uPoints*4+u*4+1] = 2.34375; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=3; ctlpoints[v*uPoints*4+u*4+0] = 3.45 ; ctlpoints[v*uPoints*4+u*4+1] = 2.3625; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=3; ctlpoints[v*uPoints*4+u*4+0] = 3.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=4; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 0.45; ctlpoints[v*uPoints*4+u*4+2] = -0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=4; ctlpoints[v*uPoints*4+u*4+0] = 3.1; ctlpoints[v*uPoints*4+u*4+1] = 0.675; ctlpoints[v*uPoints*4+u*4+2] = -0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=4; ctlpoints[v*uPoints*4+u*4+0] = 2.4 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = -0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=4; ctlpoints[v*uPoints*4+u*4+0] = 3.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=4; ctlpoints[v*uPoints*4+u*4+0] = 3.525 ; ctlpoints[v*uPoints*4+u*4+1] = 2.34375; ctlpoints[v*uPoints*4+u*4+2] = -0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=4; ctlpoints[v*uPoints*4+u*4+0] = 3.45 ; ctlpoints[v*uPoints*4+u*4+1] = 2.3625; ctlpoints[v*uPoints*4+u*4+2] = -0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=4; ctlpoints[v*uPoints*4+u*4+0] = 3.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=5; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = -0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=5; ctlpoints[v*uPoints*4+u*4+0] = 2.6; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = -0.66; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=5; ctlpoints[v*uPoints*4+u*4+0] = 2.3 ; ctlpoints[v*uPoints*4+u*4+1] = 1.95; ctlpoints[v*uPoints*4+u*4+2] = -0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=5; ctlpoints[v*uPoints*4+u*4+0] = 2.7 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=5; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = -0.25; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=5; ctlpoints[v*uPoints*4+u*4+0] = 2.9 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = -0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=5; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.15; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=6; ctlpoints[v*uPoints*4+u*4+0] = 1.7; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=6; ctlpoints[v*uPoints*4+u*4+0] = 2.6; ctlpoints[v*uPoints*4+u*4+1] = 1.275; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=6; ctlpoints[v*uPoints*4+u*4+0] = 2.3 ; ctlpoints[v*uPoints*4+u*4+1] = 1.95; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=6; ctlpoints[v*uPoints*4+u*4+0] = 2.7 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=6; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=6; ctlpoints[v*uPoints*4+u*4+0] = 2.9 ; ctlpoints[v*uPoints*4+u*4+1] = 2.325; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=6; ctlpoints[v*uPoints*4+u*4+0] = 2.8 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int v = 0; v <this->vPoints ; v++)
		{
			for (int u = 0; u < this->uPoints; u++)
			{
				ctlpoints[v*this->uPoints*4+u*4+0] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+1] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+2] *= cntlScale;
			}
		}

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==23)
	{
		// Handle:
		// Scale of the Teapot
		float cntlScale = 100;

		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=10;

		this->kdColor = Float3(0.25, 0.24, 0.80);
		this->ksColor = Float3(0.06, 0.06, 0.12);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 7;
		this->vPoints	= 7;

		float tempuKnots[11]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2};
		float tempvKnots[11]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		u=0; v=0; ctlpoints[v*uPoints*4+u*4+0] = -1.595; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=0; ctlpoints[v*uPoints*4+u*4+0] = -2.295; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=0; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=0; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=0; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.425; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=0; ctlpoints[v*uPoints*4+u*4+0] = -2.495 ; ctlpoints[v*uPoints*4+u*4+1] = 0.975; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=0; ctlpoints[v*uPoints*4+u*4+0] = -1.995 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=1; ctlpoints[v*uPoints*4+u*4+0] = -1.595; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=1; ctlpoints[v*uPoints*4+u*4+0] = -2.295; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=1; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=1; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=1; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.425; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=1; ctlpoints[v*uPoints*4+u*4+0] = -2.495 ; ctlpoints[v*uPoints*4+u*4+1] = 0.975; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=1; ctlpoints[v*uPoints*4+u*4+0] = -1.995 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=2; ctlpoints[v*uPoints*4+u*4+0] = -1.495; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=2; ctlpoints[v*uPoints*4+u*4+0] = -2.495; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=2; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=2; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=2; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=2; ctlpoints[v*uPoints*4+u*4+0] = -2.645 ; ctlpoints[v*uPoints*4+u*4+1] = 0.7875; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=2; ctlpoints[v*uPoints*4+u*4+0] = -1.895 ; ctlpoints[v*uPoints*4+u*4+1] = 0.45; ctlpoints[v*uPoints*4+u*4+2] = 0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=3; ctlpoints[v*uPoints*4+u*4+0] = -1.495; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=3; ctlpoints[v*uPoints*4+u*4+0] = -2.495; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=3; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=3; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=3; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=3; ctlpoints[v*uPoints*4+u*4+0] = -2.645 ; ctlpoints[v*uPoints*4+u*4+1] = 0.7875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=3; ctlpoints[v*uPoints*4+u*4+0] = -1.895 ; ctlpoints[v*uPoints*4+u*4+1] = 0.45; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=4; ctlpoints[v*uPoints*4+u*4+0] = -1.495; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=4; ctlpoints[v*uPoints*4+u*4+0] = -2.495; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=4; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 2.1; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=4; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=4; ctlpoints[v*uPoints*4+u*4+0] = -2.995 ; ctlpoints[v*uPoints*4+u*4+1] = 1.2; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=4; ctlpoints[v*uPoints*4+u*4+0] = -2.645 ; ctlpoints[v*uPoints*4+u*4+1] = 0.7875; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=4; ctlpoints[v*uPoints*4+u*4+0] = -1.895 ; ctlpoints[v*uPoints*4+u*4+1] = 0.45; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.595; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2.295; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.425; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=5; ctlpoints[v*uPoints*4+u*4+0] = -2.495 ; ctlpoints[v*uPoints*4+u*4+1] = 0.975; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.995 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = -0.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.595; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2.295; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.875; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.65; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2.695 ; ctlpoints[v*uPoints*4+u*4+1] = 1.425; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=6; ctlpoints[v*uPoints*4+u*4+0] = -2.495 ; ctlpoints[v*uPoints*4+u*4+1] = 0.975; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.995 ; ctlpoints[v*uPoints*4+u*4+1] = 0.75; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;


		for (int v = 0; v <this->vPoints ; v++)
		{
			for (int u = 0; u < this->uPoints; u++)
			{
				ctlpoints[v*this->uPoints*4+u*4+0] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+1] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+2] *= cntlScale;
			}
		}

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	else if (surfNum==24)
	{
		// Cap:
		// Scale of the Teapot
		float cntlScale = 100;

		this->trimmed = false;
		this->uBaseNum=10;
		this->vBaseNum=20;

		this->kdColor = Float3(0.25, 0.24, 0.80);
		this->ksColor = Float3(0.06, 0.06, 0.12);
		this->ka = 0.5;
		this->shininess = 1000;

		this->uOrder	= 4;
		this->vOrder	= 4;
		this->uPoints	= 7;
		this->vPoints	= 13;

		float tempuKnots[11]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2};
		float tempvKnots[17]={0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		int u,v;
		u=0; v=0; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3.0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=0; ctlpoints[v*uPoints*4+u*4+0] = 0.8; ctlpoints[v*uPoints*4+u*4+1] = 3.0; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=0; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=0; ctlpoints[v*uPoints*4+u*4+0] = 0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=0; ctlpoints[v*uPoints*4+u*4+0] = 0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=0; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=1; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=1; ctlpoints[v*uPoints*4+u*4+0] = 0.8; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0.45; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=1; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=1; ctlpoints[v*uPoints*4+u*4+0] = 0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0.112; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=1; ctlpoints[v*uPoints*4+u*4+0] = 0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.224; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=1; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.45; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0.8; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.112 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0.2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.224 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=2; ctlpoints[v*uPoints*4+u*4+0] = 0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0.8; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0.2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=3; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=4; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.45; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0.8; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=4; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.112 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0.2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.224 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=4; ctlpoints[v*uPoints*4+u*4+0] = -0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=5; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=5; ctlpoints[v*uPoints*4+u*4+0] = -0.8; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0.45; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=5; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=5; ctlpoints[v*uPoints*4+u*4+0] = -0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0.112; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=5; ctlpoints[v*uPoints*4+u*4+0] = -0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.224; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=5; ctlpoints[v*uPoints*4+u*4+0] = -1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=6; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=6; ctlpoints[v*uPoints*4+u*4+0] = -0.8; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=6; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=6; ctlpoints[v*uPoints*4+u*4+0] = -0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=6; ctlpoints[v*uPoints*4+u*4+0] = -0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=6; ctlpoints[v*uPoints*4+u*4+0] = -1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=7; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=7; ctlpoints[v*uPoints*4+u*4+0] = -0.8; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = -0.45; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=7; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=7; ctlpoints[v*uPoints*4+u*4+0] = -0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = -0.112; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=7; ctlpoints[v*uPoints*4+u*4+0] = -0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.224; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=7; ctlpoints[v*uPoints*4+u*4+0] = -1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=8; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.45; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = -0.8; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=8; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.112 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = -0.2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.224 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=8; ctlpoints[v*uPoints*4+u*4+0] = -0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = -0.8; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = -0.2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=9; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.45; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = -0.8; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.112 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = -0.2; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.224 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.4; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=10; ctlpoints[v*uPoints*4+u*4+0] = 0.728 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -1.3; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=11; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=11; ctlpoints[v*uPoints*4+u*4+0] = 0.8; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = -0.45; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=11; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=11; ctlpoints[v*uPoints*4+u*4+0] = 0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = -0.112; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=11; ctlpoints[v*uPoints*4+u*4+0] = 0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.224; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = -0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=11; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = -0.728; ctlpoints[v*uPoints*4+u*4+3] = 1;

		u=0; v=12; ctlpoints[v*uPoints*4+u*4+0] = 0; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=1; v=12; ctlpoints[v*uPoints*4+u*4+0] = 0.8; ctlpoints[v*uPoints*4+u*4+1] = 3; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=2; v=12; ctlpoints[v*uPoints*4+u*4+0] = 0 ; ctlpoints[v*uPoints*4+u*4+1] = 2.7; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=3; v=12; ctlpoints[v*uPoints*4+u*4+0] = 0.2 ; ctlpoints[v*uPoints*4+u*4+1] = 2.55; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=4; v=12; ctlpoints[v*uPoints*4+u*4+0] = 0.4 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=5; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.4; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;
		u=6; v=12; ctlpoints[v*uPoints*4+u*4+0] = 1.3 ; ctlpoints[v*uPoints*4+u*4+1] = 2.25; ctlpoints[v*uPoints*4+u*4+2] = 0; ctlpoints[v*uPoints*4+u*4+3] = 1;

		for (int v = 0; v <this->vPoints ; v++)
		{
			for (int u = 0; u < this->uPoints; u++)
			{
				ctlpoints[v*this->uPoints*4+u*4+0] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+1] *= cntlScale;
				ctlpoints[v*this->uPoints*4+u*4+2] *= cntlScale;
			}
		}

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	///////////////////////
	else
	{
		this->uBaseNum=10;
		this->vBaseNum=5;

		this->kdColor = Float3(0.48, 0.24, 0.64);
		this->ksColor = Float3(0.12, 0.06, 0.16);
		this->ka = 0.5;
		this->shininess = 10;

		//		Transform t;
		//		t.type	= 1;
		//		t.translate  = Float3(0,0,1);
		//		this->transforms.push_back(t);

		this->uOrder	= 3;
		this->vOrder	= 4;
		this->uPoints	= 6;
		this->vPoints	= 4;

		float tempvKnots[10]={0,0,0,0,1,1,1,1};

		float tempuKnots[10]={0,0,0,0.25,0.5,0.75,1,1,1};

		int vPoints=this->vPoints;
		int uPoints=this->uPoints;

		ctlpoints = new float[this->uPoints*this->vPoints*4];
		uKnots = new float[this->uPoints+this->uOrder];
		vKnots = new float[this->vPoints+this->vOrder];

		for (int v = 0; v < vPoints; v++)
		{
			for (int u = 0; u < uPoints; u++)
			{
				ctlpoints[v*uPoints*4+u*4+0] = (u*2.0 - 5);
				ctlpoints[v*uPoints*4+u*4+1] = (v*2.0 - 3);
				ctlpoints[v*uPoints*4+u*4+2] = sin(PI/180*(3*v-5)*u*.001);
				ctlpoints[v*uPoints*4+u*4+3] = 1;
			}
		}

		for (int u = 0; u < this->uPoints + this->uOrder; u++)
			uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
		for (int v = 0; v < this->vPoints + this->vOrder; v++)
			vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);
	}
	this->cntlPoints	= ctlpoints;
	this->uKnotVector	= uKnots;
	this->vKnotVector	= vKnots;
	this->normalSign	= 1;
}

#pragma warning(pop)