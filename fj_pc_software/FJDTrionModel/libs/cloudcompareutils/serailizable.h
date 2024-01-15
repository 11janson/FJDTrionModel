#pragma once

#include <QJsonObject>
#include <QJsonArray>
#include <QObject>
#include "cloudcompareutils_global.h"
namespace CS {
		class TRIONMETAHUBUTILS_EXPORT Serailizable : public QObject
		{
			Q_OBJECT
		public:
		public:
			explicit Serailizable(QObject *parent = nullptr) : QObject(parent)
			{

			}
			virtual QJsonObject serailize() const = 0;
			virtual void unserailize(const QJsonObject &) = 0;
		};

}


