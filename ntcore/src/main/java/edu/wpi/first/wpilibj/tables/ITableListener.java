// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.tables;

/**
 * A listener that listens to changes in values in a {@link ITable}.
 * @deprecated Use Consumer&lt;{@link edu.wpi.first.networktables.EntryNotification}&gt;,
 * {@link edu.wpi.first.networktables.TableEntryListener}, or
 * {@link edu.wpi.first.networktables.TableListener} as appropriate.
 */
@FunctionalInterface
@Deprecated
@SuppressWarnings("checkstyle:all")
public interface ITableListener {
    /**
     * Called when a key-value pair is changed in a {@link ITable}
     * @param source the table the key-value pair exists in
     * @param key the key associated with the value that changed
     * @param value the new value
     * @param isNew true if the key did not previously exist in the table, otherwise it is false
     */
    public void valueChanged(ITable source, String key, Object value, boolean isNew);

    /**
     * Extended version of valueChanged.  Called when a key-value pair is
     * changed in a {@link ITable}.  The default implementation simply calls
     * valueChanged().  If this is overridden, valueChanged() will not be
     * called.
     * @param source the table the key-value pair exists in
     * @param key the key associated with the value that changed
     * @param value the new value
     * @param flags update flags; for example, NOTIFY_NEW if the key did not
     * previously exist in the table
     */
    default public void valueChangedEx(ITable source, String key, Object value, int flags) {
        // NOTIFY_NEW = 0x04
        valueChanged(source, key, value, (flags & 0x04) != 0);
    }
}
