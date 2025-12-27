#!/usr/bin/env python3
"""
Batch convert all .xosc files in the current directory to use 'ego_vehicle' as ego name.
- Replaces 'hero' with 'ego_vehicle' everywhere needed
- Adds/fixes role_name property
- Modifies files IN PLACE
"""

import xml.etree.ElementTree as ET
import os
import glob

def convert_file(filepath):
    try:
        tree = ET.parse(filepath)
        root = tree.getroot()
        modified = False

        entities = root.find('Entities')
        if entities is None:
            print(f"⚠️  Skipped {os.path.basename(filepath)}: No <Entities>")
            return False

        ego_obj = None
        for obj in entities.findall('ScenarioObject'):
            name = obj.get('name', '')
            vehicle = obj.find('Vehicle')
            is_ego_by_type = False
            if vehicle is not None:
                props = vehicle.find('Properties')
                if props is not None:
                    for prop in props.findall('Property'):
                        if prop.get('name') == 'type' and prop.get('value') == 'ego_vehicle':
                            is_ego_by_type = True
                            break

            if name == 'hero' or is_ego_by_type:
                ego_obj = obj
                if name != 'ego_vehicle':
                    obj.set('name', 'ego_vehicle')
                    print(f"  → Renamed ScenarioObject '{name}' → 'ego_vehicle'")
                    modified = True
                break

        if ego_obj is None:
            print(f"ℹ️  Skipped {os.path.basename(filepath)}: No ego vehicle found")
            return False

        # Step 2: Fix role_name ONLY if it's a Vehicle
        vehicle = ego_obj.find('Vehicle')
        if vehicle is not None:
            props = vehicle.find('Properties')
            if props is None:
                props = ET.SubElement(vehicle, 'Properties')

            # Remove existing role_name
            for prop in list(props.findall('Property')):
                if prop.get('name') == 'role_name':
                    props.remove(prop)
                    modified = True

            # Add correct role_name
            role_prop = ET.SubElement(props, 'Property')
            role_prop.set('name', 'role_name')
            role_prop.set('value', 'ego_vehicle')
            modified = True
        else:
            print(f"  → Note: Ego object is not a Vehicle (e.g., Pedestrian). role_name not applicable.")

        # Step 3: Replace all entityRef="hero"
        for elem in root.iter():
            if elem.get('entityRef') == 'hero':
                elem.set('entityRef', 'ego_vehicle')
                modified = True

        # Also replace masterEntityRef="hero" (you missed this!)
        for elem in root.iter():
            if elem.get('masterEntityRef') == 'hero':
                elem.set('masterEntityRef', 'ego_vehicle')
                modified = True

        if modified:
            # Preserve XML declaration and encoding
            tree.write(filepath, encoding='UTF-8', xml_declaration=True)
            print(f"✅ Updated: {os.path.basename(filepath)}")
            return True
        else:
            print(f"⏭️  Already OK: {os.path.basename(filepath)}")
            return False

    except Exception as e:
        print(f"❌ Error processing {filepath}: {e}")
        return False


def main():
    xosc_files = glob.glob("*.xosc")
    if not xosc_files:
        print("📭 No .xosc files found in current directory.")
        return

    print(f"🔍 Found {len(xosc_files)} .xosc file(s). Converting to 'ego_vehicle' style...\n")
    updated = 0
    for f in sorted(xosc_files):
        if convert_file(f):
            updated += 1

    print(f"\n✨ Done. {updated}/{len(xosc_files)} file(s) updated.")


if __name__ == '__main__':
    main()