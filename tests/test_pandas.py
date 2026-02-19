#!/usr/bin/env python3
"""
Test script for pandas functionality
Tests basic pandas operations and data manipulation
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

def test_pandas_basic():
    """Test basic pandas DataFrame operations"""
    print("🐼 Testing pandas basic operations...")
    
    # Create sample data
    data = {
        'Name': ['Alice', 'Bob', 'Charlie', 'Diana', 'Eve'],
        'Age': [25, 30, 35, 28, 32],
        'Score': [85.5, 92.3, 78.9, 88.7, 91.2],
        'City': ['New York', 'London', 'Tokyo', 'Paris', 'Berlin']
    }
    
    # Create DataFrame
    df = pd.DataFrame(data)
    print("✅ DataFrame created successfully")
    print(f"   Shape: {df.shape}")
    print(f"   Columns: {list(df.columns)}")
    
    # Basic operations
    print(f"   Average age: {df['Age'].mean():.1f}")
    print(f"   Average score: {df['Score'].mean():.1f}")
    print(f"   Max score: {df['Score'].max()}")
    
    return df

def test_pandas_analysis(df):
    """Test pandas data analysis features"""
    print("\n📊 Testing pandas data analysis...")
    
    # Statistical summary
    print("✅ Statistical summary:")
    print(df.describe())
    
    # Filtering
    high_scorers = df[df['Score'] > 85]
    print(f"\n✅ High scorers (>85): {len(high_scorers)} people")
    print(high_scorers[['Name', 'Score']].to_string(index=False))
    
    # Grouping
    age_groups = df.groupby(df['Age'] > 30)['Score'].mean()
    print(f"\n✅ Average score by age group:")
    print(f"   Age ≤ 30: {age_groups[False]:.1f}")
    print(f"   Age > 30: {age_groups[True]:.1f}")
    
    return high_scorers

def test_pandas_file_operations():
    """Test pandas file I/O operations"""
    print("\n💾 Testing pandas file operations...")
    
    script_dir = Path(__file__).parent
    
    # Create time series data
    dates = pd.date_range('2024-01-01', periods=100, freq='D')
    ts_data = pd.DataFrame({
        'date': dates,
        'temperature': 20 + 10 * np.sin(np.arange(100) * 2 * np.pi / 365) + np.random.normal(0, 2, 100),
        'humidity': 50 + 20 * np.cos(np.arange(100) * 2 * np.pi / 365) + np.random.normal(0, 5, 100)
    })
    
    # Save to CSV
    csv_path = script_dir / "sample_data.csv"
    ts_data.to_csv(csv_path, index=False)
    print(f"✅ Data saved to: {csv_path}")
    
    # Read back from CSV
    loaded_data = pd.read_csv(csv_path)
    print(f"✅ Data loaded from CSV, shape: {loaded_data.shape}")
    
    # Save to Excel (if openpyxl is available)
    try:
        excel_path = script_dir / "sample_data.xlsx"
        ts_data.to_excel(excel_path, index=False, engine='openpyxl')
        print(f"✅ Data saved to Excel: {excel_path}")
    except ImportError:
        print("⚠️  Excel export not available (openpyxl not installed)")
    
    return ts_data

def test_pandas_visualization(ts_data):
    """Test pandas with matplotlib visualization"""
    print("\n📈 Testing pandas visualization...")
    
    script_dir = Path(__file__).parent
    
    # Create plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Temperature plot
    ts_data.plot(x='date', y='temperature', ax=ax1, title='Temperature Over Time', color='red')
    ax1.set_ylabel('Temperature (°C)')
    ax1.grid(True, alpha=0.3)
    
    # Humidity plot
    ts_data.plot(x='date', y='humidity', ax=ax2, title='Humidity Over Time', color='blue')
    ax2.set_ylabel('Humidity (%)')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    plot_path = script_dir / "pandas_test_plot.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Visualization saved to: {plot_path}")

def main():
    """Main test function"""
    print("🧪 Pandas Test Suite")
    print("=" * 50)
    
    try:
        # Test basic operations
        df = test_pandas_basic()
        
        # Test analysis
        high_scorers = test_pandas_analysis(df)
        
        # Test file operations
        ts_data = test_pandas_file_operations()
        
        # Test visualization
        test_pandas_visualization(ts_data)
        
        print("\n🎉 All pandas tests completed successfully!")
        print("=" * 50)
        print(f"✅ Pandas version: {pd.__version__}")
        print(f"✅ NumPy version: {np.__version__}")
        print("✅ Basic operations: PASSED")
        print("✅ Data analysis: PASSED")
        print("✅ File I/O: PASSED")
        print("✅ Visualization: PASSED")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()