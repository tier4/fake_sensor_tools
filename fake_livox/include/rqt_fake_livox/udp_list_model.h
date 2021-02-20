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
 * @file udp_list_model.h
 * @brief UDP list model class
 */

#pragma once

#include <udp_info.h>
#include <string>
#include <vector>

#include <QAbstractTableModel>

class UDPListModel : public QAbstractTableModel
{
public:
  /**
   * @brief Constructor.
   * @param[in] parent QObject
   */
  explicit UDPListModel(QObject * parent = nullptr);

  /**
   * @brief Destructor.
   */
  ~UDPListModel();

  /**
   * @brief Returns the item flags for the given index.
   * @param[in] index index
   * @return item flags
   */
  Qt::ItemFlags flags(const QModelIndex & index) const override;

  /**
   * @brief Returns the number of columns for the children of the given parent.
   * @param[in] parent QObject
   * @return number of columns
   */
  int columnCount(const QModelIndex & parent = QModelIndex()) const override;

  /**
   * @brief Returns the number of rows under the given parent.
   * @param[in] parent QObject
   * @return number of rows
   */
  int rowCount(const QModelIndex & parent = QModelIndex()) const override;

  /**
   * @brief Returns the data stored under the given role for the item referred to by the index.
   * @param[in] index index
   * @param[in] role role
   * @return data stored under the given role
   */
  QVariant data(const QModelIndex & index, int role = Qt::DisplayRole) const override;

  /**
   * @brief Sets the role data for the item at index to value.
   * @param[in] index index
   * @param[in] value value
   * @param[in] role role
   * @return true if successful; otherwise returns false
   */
  bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::DisplayRole) override;

  /**
   * @brief Returns the data for the given role and section in the header with the specified orientation.
   * @param[in] section section
   * @param[in] orientation orientation
   * @param[in] role role
   * @return data for the given role and section
   */
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

  /**
   * @brief Add item.
   * @param[in] info UDP packet information
   */
  void add(const UDPInfo & info);

  /**
   * @brief Remove all items.
   */
  void removeAll();

  /**
   * @brief Toggle transmit or not.
   * @param[in] index index
   */
  void toggleTransmit(const QModelIndex & index);

  /**
   * @brief Get transmit or not.
   * @param[in] index index
   * @return transmit or not
   */
  bool getTransmit(const QModelIndex & index);

  /**
   * @brief Get filter program.
   * @return filter program
   */
  std::string getFileter();

private:
  std::vector<UDPInfo> list_;  //!< @brief list of UDP packet information
};
