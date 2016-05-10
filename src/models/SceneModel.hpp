#pragma once

#include <QAbstractListModel>
#include "models/Scene.hpp"

namespace meshroom
{

class SceneModel : public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(int count READ rowCount NOTIFY countChanged)

public:
    enum SceneRoles
    {
        UrlRole = Qt::UserRole + 1,
        NameRole,
        DateRole,
        UserRole,
        ThumbnailRole,
        DirtyRole,
        ModelDataRole
    };

public:
    SceneModel(QObject* parent = 0);
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addScene(Scene* scene);

public:
    Q_SLOT void addScene(const QUrl& url);
    Q_SLOT void removeScene(Scene* scene);
    Q_SLOT QVariantMap get(int row) const;
    Q_SIGNAL void countChanged(int c);

protected:
    QHash<int, QByteArray> roleNames() const override;

private:
    QList<Scene*> _scenes;
};

} // namespace