/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file udp_list_model.cpp
 * @brief UDP list model class
 */

#include <fmt/format.h>

#include <rqt_fake_livox/udp_list_model.h>

UDPListModel::UDPListModel(QObject * parent) : QAbstractTableModel(parent), enabled_(false) {}

UDPListModel::~UDPListModel() {}

Qt::ItemFlags UDPListModel::flags(const QModelIndex & index) const
{
  Qt::ItemFlags result = QAbstractTableModel::flags(index);
  if (index.column() == 0) {
    result |= Qt::ItemIsUserCheckable;
  }
  return result;
}

int UDPListModel::columnCount(const QModelIndex & parent) const {
  (void)parent;
  return 6;
}

int UDPListModel::rowCount(const QModelIndex & parent) const {
  (void)parent;
  return list_.size();
}

QVariant UDPListModel::data(const QModelIndex & index, int role) const
{
  if (index.isValid()) {
    if (role == Qt::CheckStateRole && index.column() == 0) {
      if (enabled_) {
        return list_[index.row()].getTransmit();
      }
      return QVariant();
    }

    if (role == Qt::DisplayRole) {
      switch (index.column()) {
        case 1:
          return list_[index.row()].getSourceAddress();
        case 2:
          return list_[index.row()].getSourcePort();
        case 3:
          return list_[index.row()].getDestAddress();
        case 4:
          return list_[index.row()].getDestPort();
        case 5:
          return list_[index.row()].getPacketCount();
        default:
          break;
      }
    } else if (role == Qt::TextAlignmentRole) {
      switch (index.column()) {
        case 2:
        case 4:
        case 5:
          return QVariant(Qt::AlignRight | Qt::AlignVCenter);
        default:
          break;
      }
    }
  }

  return QVariant();
}

bool UDPListModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
  (void)value;

  if (!index.isValid()) return false;

  if (role == Qt::CheckStateRole && index.column() == 0) {
    list_[index.row()].toggleTransmit();
    return true;
  }

  return false;
}

QVariant UDPListModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (role != Qt::DisplayRole) {
    return QVariant();
  }
  if (orientation == Qt::Horizontal) {
    switch (section) {
      case 0:
        return tr("");
      case 1:
        return tr("Source address");
      case 2:
        return tr("Source port");
      case 3:
        return tr("Dest address");
      case 4:
        return tr("Dest port");
      case 5:
        return tr("Packets");
      default:
        return QVariant();
    }
  } else {
    return QVariant(section + 1);
  }

  return QVariant();
}

void UDPListModel::add(const UDPInfo & info)
{
  for (auto & list_info : list_) {
    if (list_info == info) {
      list_info.incrementPacketCount();
      return;
    }
  }

  beginInsertRows(QModelIndex(), list_.size(), list_.size());
  list_.push_back(info);
  endInsertRows();
}

void UDPListModel::removeAll() { list_.clear(); }

void UDPListModel::toggleTransmit(const QModelIndex & index)
{
  if (enabled_) list_[index.row()].toggleTransmit();
}

bool UDPListModel::getTransmit(const QModelIndex & index) { return list_[index.row()].getTransmit(); }

std::string UDPListModel::getFilter()
{
  std::string ret = "";

  for (const auto & info : list_) {
    if (info.getTransmit()) {
      if (!ret.empty()) ret += " or ";
      ret += fmt::format(
        "(src {} and src port {} and dst {} and dst port {})", info.getSourceAddress().toStdString(),
        info.getSourcePort(), info.getDestAddress().toStdString(), info.getDestPort());
    }
  }
  return ret;
}

void UDPListModel::setEnabled(bool enabled) { enabled_ = enabled; }
