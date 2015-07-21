/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael Koval <mkoval@cs.cmu.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include "dart/utils/SchemaResourceRetriever.h"
#include "TestHelpers.h"

using dart::utils::Resource;
using dart::utils::ResourcePtr;
using dart::utils::ResourceRetriever;
using dart::utils::SchemaResourceRetriever;

TEST(SchemaResourceRetriever, exists_NothingRegistered_ReturnsFalse)
{
  SchemaResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists("package://test/foo"));
}

TEST(SchemaResourceRetriever, exists_AllRetrieversFail_ReturnsFalse)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  SchemaResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);

  EXPECT_FALSE(retriever.exists("package://test/foo"));

  EXPECT_TRUE(retriever1->mRetrieve.empty());
  ASSERT_EQ(1, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mRetrieve.empty());
  ASSERT_EQ(1, retriever2->mExists.size());
  EXPECT_EQ("package://test/foo", retriever2->mExists.front());
}

TEST(SchemaResourceRetriever, exists_SchemaResourceRetrieverSucceeds_ReturnsTrue)
{
  auto retriever1 = std::make_shared<PresentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  SchemaResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever2));
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(retriever.exists("package://test/foo"));

  EXPECT_TRUE(retriever1->mRetrieve.empty());
  ASSERT_EQ(1, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  EXPECT_TRUE(retriever2->mRetrieve.empty());
  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(SchemaResourceRetriever, exists_DefaultResourceRetrieverSucceeds_ReturnsTrue)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<PresentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  SchemaResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(retriever.exists("package://test/foo"));
  EXPECT_TRUE(retriever1->mRetrieve.empty());
  ASSERT_EQ(1, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mRetrieve.empty());
  ASSERT_EQ(1, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(SchemaResourceRetriever, retrieve_NothingRegistered_ReturnsNull)
{
  SchemaResourceRetriever retriever;
  EXPECT_EQ(nullptr, retriever.retrieve("package://test/foo"));
}

TEST(SchemaResourceRetriever, retrieve_AllRetrieversFail_ReturnsNull)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  SchemaResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);

  EXPECT_EQ(nullptr, retriever.retrieve("package://test/foo"));

  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mRetrieve.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1, retriever2->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever2->mRetrieve.front());
}

TEST(SchemaResourceRetriever, retrieve_SchemaResourceRetrieverSucceeds_ReturnsNonNull)
{
  auto retriever1 = std::make_shared<PresentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  SchemaResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever2));
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(nullptr != retriever.retrieve("package://test/foo"));

  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  EXPECT_TRUE(retriever2->mRetrieve.empty());
  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(SchemaResourceRetriever, retrieve_DefaultResourceRetrieverSucceeds_ReturnsNonNull)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<PresentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  SchemaResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(nullptr != retriever.retrieve("package://test/foo"));
  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mRetrieve.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mRetrieve.front());

  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}