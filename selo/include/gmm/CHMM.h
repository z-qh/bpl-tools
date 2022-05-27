/***************************************************************************
Module Name:
	Continuous Observation Hidden Markov Model with Gaussian Mixture

History:
	2003/12/13	Fei Wang
	2013 luxiaoxun
***************************************************************************/

#pragma once
#include <vector>
#include <cassert>
#include <cstring>
#include "GMM.h"

class CHMM
{
public:
	CHMM(int stateNum = 1, int dimNum = 1, int mixNum = 1);
	~CHMM();

	int GetStateNum()	    { return m_stateNum; }
	int GetMaxIterNum()		{ return m_maxIterNum; }
	double GetEndError()	{ return m_endError; }

	void SetMaxIterNum(int n)	{ m_maxIterNum = n; }
	void SetEndError(double e)	{ m_endError = e; }

	GMM* GetStateModel(int i);
	double GetStateInit(int i);
	double GetStateFinal(int i);
	double GetStateTrans(int i, int j);

	void Zero();
	void Norm();

	double GetProbability(std::vector<double*>& seq);
	//Viterbi Decode
	double Decode(std::vector<double*>& seq, std::vector<int>& state);

	/*	SampleFile: <size><dim><seq_size><seq_data>...<seq_size><seq_data>...
	    size: number of samples
        dim: dimension of feature
        seq_size: number of each sample's feature
	*/
	void Train(const char* sampleFileName);
	void Init(const char* sampleFileName);
	double getTransProb(int i,int j);

	friend std::ostream& operator<<(std::ostream& out, CHMM& hmm);
	friend std::istream& operator>>(std::istream& in, CHMM& hmm);
    //�����������ı��ļ�ת��Ϊ�������ļ�
	void TextTransform(const char* InputText, const char * OutputBinaryText);

private:
	int m_stateNum;        //״̬��
	GMM** m_stateModel;    //��˹���ģ��
	double* m_stateInit;   //״̬��ʼ����
	double** m_stateTran;  //״̬ת�Ƹ���

	int m_maxIterNum;      // ����������
	double m_endError;

	void Allocate(int state, int dim = 1, int mix = 1);
	void Dispose();

	double LogProb(double p);
	void DumpSampleFile(const char* fileName);
};
