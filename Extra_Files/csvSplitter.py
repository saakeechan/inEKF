import os
import pandas as pd
from pathlib import Path

def split_csv_by_time_gaps(input_dir, output_dir, time_threshold=0.025):
    """
    Split CSV files based on time gaps.
    
    Args:
        input_dir: Directory containing input CSV files
        output_dir: Directory to save split CSV files
        time_threshold: Maximum time difference before splitting (default: 0.025)
    """
    # Create output directory if it doesn't exist
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Get all CSV files from input directory
    input_path = Path(input_dir)
    csv_files = list(input_path.glob('*.csv'))
    
    if not csv_files:
        print(f"No CSV files found in {input_dir}")
        return
    
    print(f"Found {len(csv_files)} CSV files to process")
    
    # Process each CSV file
    for csv_file in csv_files:
        print(f"\nProcessing: {csv_file.name}")
        
        try:
            # Read the CSV file
            df = pd.read_csv(csv_file)
            
            # Find the time column (assuming it's named 'time' or similar)
            time_column = None
            for col in df.columns:
                if 'time' in col.lower():
                    time_column = col
                    break
            
            if time_column is None:
                print(f"  Warning: No time column found in {csv_file.name}. Skipping.")
                continue
            
            print(f"  Using time column: {time_column}")
            
            # Calculate time differences
            time_diffs = df[time_column].diff()
            
            # Find indices where time difference exceeds threshold
            split_indices = [0]  # Start with index 0
            for i in range(1, len(time_diffs)):
                if pd.notna(time_diffs.iloc[i]) and time_diffs.iloc[i] > time_threshold:
                    split_indices.append(i)
            split_indices.append(len(df))  # Add end index
            
            # Split and save the dataframe
            num_runs = len(split_indices) - 1
            original_filename = csv_file.stem  # Filename without extension
            
            for run_num in range(num_runs):
                start_idx = split_indices[run_num]
                end_idx = split_indices[run_num + 1]
                
                # Extract the segment
                segment_df = df.iloc[start_idx:end_idx]
                
                # Create output filename
                output_filename = f"{original_filename}_run{run_num + 1}.csv"
                output_filepath = output_path / output_filename
                
                # Save the segment
                segment_df.to_csv(output_filepath, index=False)
                print(f"  Saved: {output_filename} ({len(segment_df)} rows)")
            
            print(f"  Total runs created: {num_runs}")
            
        except Exception as e:
            print(f"  Error processing {csv_file.name}: {str(e)}")
            continue

if __name__ == "__main__":
    # Get the directory of this script
    script_dir = Path(__file__).parent
    
    # Define input and output directories
    input_dir = script_dir / ".." / "Data"
    output_dir = script_dir / ".." / "splitData"
    
    print("CSV Splitter")
    print("=" * 50)
    print(f"Input directory: {input_dir.resolve()}")
    print(f"Output directory: {output_dir.resolve()}")
    print(f"Time threshold: 0.025")
    print("=" * 50)
    
    # Run the splitter
    split_csv_by_time_gaps(input_dir, output_dir)