/*
 * Original work Copyright 2009 - 2010 Kevin Ackley (kackley@gwi.net)
 * Modified work Copyright 2018 - 2020 Andy Maloney <asmaloney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <stack>

#include <xercesc/sax/InputSource.hpp>
#include <xercesc/sax2/DefaultHandler.hpp>

#include "Common.h"

using namespace XERCES_CPP_NAMESPACE;

namespace XERCES_CPP_NAMESPACE
{
   class SAX2XMLReader;
}

namespace e57
{
   class CheckedFile;

   class E57XmlParser : public DefaultHandler
   {
   public:
      E57XmlParser( ImageFileImplSharedPtr imf );
      ~E57XmlParser() override;

      void init();

      void parse( InputSource &inputSource );

   private:
      /// SAX interface
      void startElement( const XMLCh *const uri, const XMLCh *const localName, const XMLCh *const qName,
                         const Attributes &attributes ) override;
      void endElement( const XMLCh *const uri, const XMLCh *const localName, const XMLCh *const qName ) override;
      void characters( const XMLCh *const chars, const XMLSize_t length ) override;

      /// SAX error interface
      void warning( const SAXParseException &ex ) override;
      void error( const SAXParseException &ex ) override;
      void fatalError( const SAXParseException &ex ) override;

      ustring toUString( const XMLCh *const xml_str );
      ustring lookupAttribute( const Attributes &attributes, const XMLCh *attribute_name );
      bool isAttributeDefined( const Attributes &attributes, const XMLCh *attribute_name );

      ImageFileImplSharedPtr imf_; /// Image file we are reading

      struct ParseInfo
      {
         /// All the fields need to remember while parsing the XML
         /// Not all fields are used at same time, depends on node type
         /// Needed because not all info is available at one time to create the
         /// node.
         NodeType nodeType;               // used by all types
         int64_t minimum;                 // used in E57_INTEGER, E57_SCALED_INTEGER
         int64_t maximum;                 // used in E57_INTEGER, E57_SCALED_INTEGER
         double scale;                    // used in E57_SCALED_INTEGER
         double offset;                   // used in E57_SCALED_INTEGER
         FloatPrecision precision;        // used in E57_FLOAT
         double floatMinimum;             // used in E57_FLOAT
         double floatMaximum;             // used in E57_FLOAT
         int64_t fileOffset;              // used in E57_BLOB, E57_COMPRESSED_VECTOR
         int64_t length;                  // used in E57_BLOB
         bool allowHeterogeneousChildren; // used in E57_VECTOR
         int64_t recordCount;             // used in E57_COMPRESSED_VECTOR
         ustring childText;               // used by all types, accumulates all child text between tags

         /// Holds node for Structure, Vector, and CompressedVector so can append
         /// child elements
         NodeImplSharedPtr container_ni;

         ParseInfo(); // default ctor
         void dump( int indent = 0, std::ostream &os = std::cout ) const;
      };
      std::stack<ParseInfo> stack_; /// Stores the current path in tree we are reading

      SAX2XMLReader *xmlReader;
   };

   class E57XmlFileInputSource : public InputSource
   {
   public:
      E57XmlFileInputSource( CheckedFile *cf, uint64_t logicalStart, uint64_t logicalLength );
      ~E57XmlFileInputSource() override = default;

      E57XmlFileInputSource( const E57XmlFileInputSource & ) = delete;
      E57XmlFileInputSource &operator=( const E57XmlFileInputSource & ) = delete;

      BinInputStream *makeStream() const override;

   private:
      //??? lifetime of cf_ must be longer than this object!
      CheckedFile *cf_;
      uint64_t logicalStart_;
      uint64_t logicalLength_;
   };
}
