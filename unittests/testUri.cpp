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
#include "dart/common/Uri.h"
#include "TestHelpers.h"

using dart::common::Uri;

TEST(UriHelpers, fromString_ValidUri_ReturnsTrue)
{
  // These examples are from Section 1.1.2 of RFC 3986.
  Uri uri;

  ASSERT_TRUE(uri.fromString("ftp://ftp.is.co.za/rfc/rfc1808.txt"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("ftp", *uri.mScheme);
  EXPECT_EQ("ftp.is.co.za", *uri.mAuthority);
  EXPECT_EQ("/rfc/rfc1808.txt", *uri.mPath);

  ASSERT_TRUE(uri.fromString("http://www.ietf.org/rfc/rfc2396.txt"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("http", *uri.mScheme);
  EXPECT_EQ("www.ietf.org", *uri.mAuthority);
  EXPECT_EQ("/rfc/rfc2396.txt", *uri.mPath);

  ASSERT_TRUE(uri.fromString("ldap://[2001:db8::7]/c=GB?objectClass?one"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_TRUE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("ldap", *uri.mScheme);
  EXPECT_EQ("[2001:db8::7]", *uri.mAuthority);
  EXPECT_EQ("/c=GB", *uri.mPath);
  EXPECT_EQ("objectClass?one", *uri.mQuery);

  ASSERT_TRUE(uri.fromString("mailto:John.Doe@example.com"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("mailto", *uri.mScheme);
  EXPECT_EQ("John.Doe@example.com", *uri.mPath);

  ASSERT_TRUE(uri.fromString("news:comp.infosystems.www.servers.unix"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("news", *uri.mScheme);
  EXPECT_EQ("comp.infosystems.www.servers.unix", *uri.mPath);

  ASSERT_TRUE(uri.fromString("tel:+1-816-555-1212"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("tel", *uri.mScheme);
  EXPECT_EQ("+1-816-555-1212", *uri.mPath);

  ASSERT_TRUE(uri.fromString("telnet://192.0.2.16:80/"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("telnet", *uri.mScheme);
  EXPECT_EQ("192.0.2.16:80", *uri.mAuthority);
  EXPECT_EQ("/", *uri.mPath);

  ASSERT_TRUE(uri.fromString("urn:oasis:names:specification:docbook:dtd:xml:4.1.2"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_EQ("oasis:names:specification:docbook:dtd:xml:4.1.2", *uri.mPath);
}

TEST(UriHelpers, fromStringOrPath_PathNotUri_ReturnsOnlyPath)
{
  Uri uri;

  ASSERT_TRUE(uri.fromStringOrPath("foo"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("foo", *uri.mPath);

  ASSERT_TRUE(uri.fromStringOrPath("foo/"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("foo/", *uri.mPath);

  ASSERT_TRUE(uri.fromStringOrPath("foo/bar"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("foo/bar", *uri.mPath);

  ASSERT_TRUE(uri.fromStringOrPath("/foo"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("/foo", *uri.mPath);

  ASSERT_TRUE(uri.fromStringOrPath("/foo/"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("/foo/", *uri.mPath);

  ASSERT_TRUE(uri.fromStringOrPath("/foo/bar"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("/foo/bar", *uri.mPath);
}

TEST(UriHelpers, fromStringOrPath_InputIsUri_DoesNotChange)
{
  Uri uri;

  ASSERT_TRUE(uri.fromStringOrPath("ftp://ftp.is.co.za/rfc/rfc1808.txt"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("ftp", *uri.mScheme);
  EXPECT_EQ("ftp.is.co.za", *uri.mAuthority);
  EXPECT_EQ("/rfc/rfc1808.txt", *uri.mPath);
}

TEST(UriHelpers, getUri_InputIsUri_DoesNotChange)
{
  std::vector<std::string> testUris = {
    "ftp://ftp.is.co.za/rfc/rfc1808.txt",
    "http://www.ietf.org/rfc/rfc2396.txt",
    "ldap://[2001:db8::7]/c=GB?objectClass?one",
    "mailto:John.Doe@example.com",
    "news:comp.infosystems.www.servers.unix",
    "tel:+1-816-555-1212",
    "telnet://192.0.2.16:80/",
    "urn:oasis:names:specification:docbook:dtd:xml:4.1.2"
  };

  for (const std::string& testUri : testUris)
    EXPECT_EQ(testUri, Uri::getUri(testUri));
}

TEST(UriHelpers, getUri_InputIsPath_AppendsFileSchema)
{
  std::vector<std::string> testPaths = {
    "foo",
    "foo/",
    "foo/bar",
    "/foo",
    "/foo/",
    "/foo/bar"
  };

  for(const std::string& testPath : testPaths)
  {
    const std::string testUri = "file://" + testUri;
    EXPECT_EQ(testUri, Uri::getUri(testUri));
  }
}

TEST(UriHelpers, getRelativeUri)
{
  std::vector<std::pair<std::string, std::string> > testPairs = {
  // RFC 3986, Section 5.4.1.: Normal Examples
    { "g:h"           ,  "g:h" },
    { "g"             ,  "http://a/b/c/g" },
    { "./g"           ,  "http://a/b/c/g" },
    { "g/"            ,  "http://a/b/c/g/" },
    { "/g"            ,  "http://a/g" },
    { "//g"           ,  "http://g" },
    { "?y"            ,  "http://a/b/c/d;p?y" },
    { "g?y"           ,  "http://a/b/c/g?y" },
    { "#s"            ,  "http://a/b/c/d;p?q#s" },
    { "g#s"           ,  "http://a/b/c/g#s" },
    { "g?y#s"         ,  "http://a/b/c/g?y#s" },
    { ";x"            ,  "http://a/b/c/;x" },
    { "g;x"           ,  "http://a/b/c/g;x" },
    { "g;x?y#s"       ,  "http://a/b/c/g;x?y#s" },
    { ""              ,  "http://a/b/c/d;p?q" },
    { "."             ,  "http://a/b/c/" },
    { "./"            ,  "http://a/b/c/" },
    { ".."            ,  "http://a/b/" },
    { "../"           ,  "http://a/b/" },
    { "../g"          ,  "http://a/b/g" },
    { "../.."         ,  "http://a/" },
    { "../../"        ,  "http://a/" },
    { "../../g"       ,  "http://a/g" },
  // RFC 3986, Section 5.4.2.: Abnormal Examples
    { "../../../g"    ,  "http://a/g" },
    { "../../../../g" ,  "http://a/g" },
    { "/./g"          ,  "http://a/g" },
    { "/../g"         ,  "http://a/g" },
    { "g."            ,  "http://a/b/c/g." },
    { ".g"            ,  "http://a/b/c/.g" },
    { "g.."           ,  "http://a/b/c/g.." },
    { "..g"           ,  "http://a/b/c/..g" },
    { "./../g"        ,  "http://a/b/g" },
    { "./g/."         ,  "http://a/b/c/g/" },
    { "g/./h"         ,  "http://a/b/c/g/h" },
    { "g/../h"        ,  "http://a/b/c/h" },
    { "g;x=1/./y"     ,  "http://a/b/c/g;x=1/y" },
    { "g;x=1/../y"    ,  "http://a/b/c/y" },
    { "g?y/./x"       ,  "http://a/b/c/g?y/./x" },
    { "g?y/../x"      ,  "http://a/b/c/g?y/../x" },
    { "g#s/./x"       ,  "http://a/b/c/g#s/./x" },
    { "g#s/../x"      ,  "http://a/b/c/g#s/../x" },
  };

  Uri baseUri, relativeUri, mergedUri;
  ASSERT_TRUE(baseUri.fromString("http://a/b/c/d;p?q"));

  for (const auto& it : testPairs)
  {
    const std::string& expectedUri = it.second;

    ASSERT_TRUE(relativeUri.fromString(it.first));

    // Strict mode
    ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, true));
    EXPECT_EQ(expectedUri, mergedUri.toString());

    // Backwards compatability mode
    ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, false));
    EXPECT_EQ(expectedUri, mergedUri.toString());
  }

  // Strict mode behavior.
  ASSERT_TRUE(relativeUri.fromString("http:g"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, "http:g", true));
  EXPECT_EQ("http:g", mergedUri.toString());

  // Backwards compatability mode behavior.
  ASSERT_TRUE(relativeUri.fromString("http:g"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, "http:g", false));
  EXPECT_EQ("http://a/b/c/g", mergedUri.toString());
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}