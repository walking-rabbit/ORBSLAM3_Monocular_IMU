/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

using namespace std;

namespace ORB_SLAM3
{

// 构造函数
KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) : mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
}

// 根据关键帧的BoW，更新数据库的倒排索引
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // 为每一个word添加该KeyFrame
    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

// 关键帧被删除后，更新数据库的倒排索引
void KeyFrameDatabase::erase(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    // 每一个KeyFrame包含多个words，遍历mvInvertedFile中的这些words，然后在word中删除该KeyFrame
    for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

        for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
        {
            if (pKF == *lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

// 清空关键帧数据库
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for (std::vector<list<KeyFrame *>>::iterator vit = mvInvertedFile.begin(), vend = mvInvertedFile.end(); vit != vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame *> &lKFs = *vit;

        for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend;)
        {
            KeyFrame *pKFi = *lit;
            if (pMap == pKFi->GetMap())
            {
                lit = lKFs.erase(lit);
                // Dont delete the KF because the class Map clean all the KF when it is destroyed
            }
            else
            {
                ++lit;
            }
        }
    }
}


bool compFirst(const pair<float, KeyFrame *> &a, const pair<float, KeyFrame *> &b)
{
    return a.first > b.first;
}

/**
 * @brief 找到N个融合候选N个回环候选
 *
 * @param[in] pKF 当前关键帧(我们要寻找这个关键帧的回环候选帧和融合候选帧)
 * @param[out] vpLoopCand 记录找到的回环候选帧
 * @param[out] vpMergeCand 记录找到的融合候选帧
 * @param[in] nNumCandidates 期望的候选数目,即回环和候选分别应该有多少个
 */
void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nNumCandidates)
{
    // Step 1统计与当前关键帧有相同单词的关键帧
    list<KeyFrame *> lKFsSharingWords;
    // set<KeyFrame*> spInsertedKFsSharing;
    //  当前关键帧的共视关键帧(避免将当前关键帧的共视关键帧加入回环检测)
    set<KeyFrame *> spConnectedKF;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);
        // 拿到当前关键帧的共视关键帧
        spConnectedKF = pKF->GetConnectedKeyFrames();

        // 遍历当前关键帧bow向量的每个单词
        for (DBoW2::BowVector::const_iterator vit = pKF->mBowVec.begin(), vend = pKF->mBowVec.end(); vit != vend; vit++)
        { // 拿到当前单词的逆向索引(所有有当前单词的关键帧)
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];
            // 遍历每个有该单词的关键帧
            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                /*if(spConnectedKF.find(pKFi) != spConnectedKF.end())
                {
                    continue;
                }*/
                // 如果此关键帧没有被当前关键帧访问过(防止重复添加)
                if (pKFi->mnPlaceRecognitionQuery != pKF->mnId)
                {
                    // 初始化公共单词数为0
                    pKFi->mnPlaceRecognitionWords = 0;
                    // 如果该关键帧不是当前关键帧的共视关键帧
                    if (!spConnectedKF.count(pKFi))
                    {
                        // 标记该关键帧被当前关键帧访问到（也就是有公共单词）
                        pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                        // 把当前关键帧添加到有公共单词的关键帧列表中
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                // 递增该关键帧与当前关键帧的公共单词数
                pKFi->mnPlaceRecognitionWords++;
                /*if(spInsertedKFsSharing.find(pKFi) == spInsertedKFsSharing.end())
                {
                    lKFsSharingWords.push_back(pKFi);
                    spInsertedKFsSharing.insert(pKFi);
                }*/
            }
        }
    }
    // 如果没有有公共单词的关键帧,直接返回
    if (lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    // Step 2 统计所有候选帧中与当前关键帧的公共单词数最多的单词数maxCommonWords,并筛选
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        if ((*lit)->mnPlaceRecognitionWords > maxCommonWords)
            maxCommonWords = (*lit)->mnPlaceRecognitionWords;
    }
    // 取0.8倍为阀值
    int minCommonWords = maxCommonWords * 0.8f;
    // 这里的pair是 <相似度,候选帧的指针> : 记录所有大于minCommonWords的候选帧与当前关键帧的相似度
    list<pair<float, KeyFrame *>> lScoreAndMatch;
    // 只是个统计变量,貌似没有用到
    int nscores = 0;

    // Compute similarity score.
    // 对所有大于minCommonWords的候选帧计算相似度

    // 遍历所有有公共单词的候选帧
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        // 如果当前帧的公共单词数大于minCommonWords
        if (pKFi->mnPlaceRecognitionWords > minCommonWords)
        {
            nscores++; //未使用
            // 计算相似度
            float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
            // 记录该候选帧与当前帧的相似度
            pKFi->mPlaceRecognitionScore = si;
            // 记录到容器里, 每个元素是<相似度,候选帧的指针>
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }
    // 如果为空,直接返回,表示没有符合上述条件的关键帧
    if (lScoreAndMatch.empty())
        return;
    // Step 3 : 用小组得分排序得到top3总分里最高分的关键帧,作为候选帧
    // 统计以组为单位的累计相似度和组内相似度最高的关键帧, 每个pair为<小组总相似度,组内相似度最高的关键帧指针>
    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // 变量所有被lScoreAndMatch记录的pair <相似度,候选关键帧>
    for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        // 候选关键帧
        KeyFrame *pKFi = it->second;
        // 与候选关键帧共视关系最好的10个关键帧
        vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
        // 初始化最大相似度为该候选关键帧自己的相似度
        float bestScore = it->first;
        // 初始化小组累计得分为改候选关键帧自己的相似度
        float accScore = bestScore;
        // 初始化组内相似度最高的帧为该候选关键帧本身
        KeyFrame *pBestKF = pKFi;
        // 遍历与当前关键帧共视关系最好的10帧
        for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame *pKF2 = *vit;
            // 如果该关键帧没有被当前关键帧访问过(也就是没有公共单词)则跳过
            if (pKF2->mnPlaceRecognitionQuery != pKF->mnId)
                continue;
            // 累加小组总分
            accScore += pKF2->mPlaceRecognitionScore;
            // 如果大于组内最高分,则更新当前最高分记录
            if (pKF2->mPlaceRecognitionScore > bestScore)
            {
                pBestKF = pKF2;
                bestScore = pKF2->mPlaceRecognitionScore;
            }
        }
        // 统计以组为单位的累计相似度和组内相似度最高的关键帧, 每个pair为<小组总相似度,组内相似度最高的关键帧指针>
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        // 统计最高得分, 这个bestAccSocre没有用到
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // cout << "Amount of candidates: " << lAccScoreAndMatch.size() << endl;
    //  按相似度从大到小排序
    lAccScoreAndMatch.sort(compFirst);
    // 最后返回的变量, 记录回环的候选帧
    vpLoopCand.reserve(nNumCandidates);
    // 最后返回的变量, 记录融合候选帧
    vpMergeCand.reserve(nNumCandidates);
    // 避免重复添加
    set<KeyFrame *> spAlreadyAddedKF;
    // cout << "Candidates in score order " << endl;
    // for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    int i = 0;
    list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin();
    // 遍历lAccScoreAndMatch中所有的pair, 每个pair为<小组总相似度,组内相似度最高的关键帧指针>，nNumCandidates默认为3
    while (i < lAccScoreAndMatch.size() && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates))
    {
        // cout << "Accum score: " << it->first << endl;
        //  拿到候选关键帧的指针
        KeyFrame *pKFi = it->second;
        if (pKFi->isBad())
            continue;

        // 如果没有被重复添加
        if (!spAlreadyAddedKF.count(pKFi))
        {
            // 如果候选帧与当前关键帧在同一个地图里,且候选者数量还不足够
            if (pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < nNumCandidates)
            {
                // 添加到回环候选帧里
                vpLoopCand.push_back(pKFi);
            }
            // 如果候选者与当前关键帧不再同一个地图里, 且候选者数量还不足够, 且候选者所在地图不是bad
            else if (pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad())
            {
                // 添加到融合候选帧里
                vpMergeCand.push_back(pKFi);
            }
            // 防止重复添加
            spAlreadyAddedKF.insert(pKFi);
        }
        i++;
        it++;
    }
}

vector<KeyFrame *> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, Map *pMap)
{
    list<KeyFrame *> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame *> &lKFs = mvInvertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
            {
                KeyFrame *pKFi = *lit;
                if (pKFi->mnRelocQuery != F->mnId)
                {
                    pKFi->mnRelocWords = 0;
                    pKFi->mnRelocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        if ((*lit)->mnRelocWords > maxCommonWords)
            maxCommonWords = (*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->mnRelocWords > minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
            pKFi->mRelocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
    {
        KeyFrame *pKFi = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame *pBestKF = pKFi;
        for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->mnRelocQuery != F->mnId)
                continue;

            accScore += pKF2->mRelocScore;
            if (pKF2->mRelocScore > bestScore)
            {
                pBestKF = pKF2;
                bestScore = pKF2->mRelocScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;
    set<KeyFrame *> spAlreadyAddedKF;
    vector<KeyFrame *> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
    {
        const float &si = it->first;
        if (si > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (pKFi->GetMap() != pMap)
                continue;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} // namespace ORB_SLAM
